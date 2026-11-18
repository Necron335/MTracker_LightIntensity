#!/usr/bin/env python3
"""ROS 2 Extremum Seeking node for MTracker.

This node connects the geometric Transverse Function controller in
``mtracker_esc.controller`` to the real MTracker sensor/odometry topics.

Subscriptions (from the MTracker driver):
    /wheel/odom             nav_msgs/msg/Odometry
    /light_intensity        std_msgs/msg/Float32  (lux)
    /light_intensity_valid  std_msgs/msg/Bool

Publisher (to the MTracker driver):
    /cmd_vel                geometry_msgs/msg/Twist

The node:
  1. reads robot pose and light intensity;
  2. stores measurements in one excitation-period sliding window;
  3. fits a local quadratic model of light intensity versus transverse
     coordinates f_x, f_y;
  4. estimates the local gradient at f_xy = (0, 0);
  5. computes virtual input nu using the ESC gradient law;
  6. calls controller.py to convert nu into [v_x, omega, alpha_dot];
  7. publishes [v_x, omega] and integrates alpha_dot internally.

IMPORTANT:
  - Motion is disabled by default. Enable it explicitly with
    ``-p enable_motion:=true`` after checking topics and parameters.
  - The light topics have no Header/timestamp in the current MTracker driver.
    This node pairs messages using their local reception times.
  - A strict output guard stops instead of silently clipping the extended
    control. Clipping would break the exact input/output relation of the TF
    controller. Reduce rho/omega_e/gain or change limits if the guard trips.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    # Recommended filename inside the package: controller.py
    from mtracker_esc.controller import (
        calculate_components,
        extended_control,
        integrate_alpha,
        physical_control,
        se2_compose,
        se2_inverse,
    )
except ImportError:
    # Backward compatibility if the module was kept under its earlier name.
    from mtracker_esc.kontroler import (
        calculate_components,
        extended_control,
        integrate_alpha,
        physical_control,
        se2_compose,
        se2_inverse,
    )


class MTrackerExtremumSeekingNode(Node):
    """ROS 2 node joining MTracker measurements with the TF controller."""

    def __init__(self) -> None:
        super().__init__("mtracker_extremum_seeking")

        # ---------------------------- ROS parameters -------------------------
        self.odom_topic = str(
            self.declare_parameter("odom_topic", "/wheel/odom").value
        )
        self.light_topic = str(
            self.declare_parameter("light_topic", "/light_intensity").value
        )
        self.light_valid_topic = str(
            self.declare_parameter(
                "light_valid_topic", "/light_intensity_valid"
            ).value
        )
        self.cmd_topic = str(
            self.declare_parameter("cmd_topic", "/cmd_vel").value
        )

        # Transverse-function parameters. rho values are in metres;
        # omega_e is in rad/s; epsilon2 is an angle in radians.
        self.epsilon1 = float(self.declare_parameter("epsilon1", 0.10).value)
        self.epsilon2 = float(
            self.declare_parameter("epsilon2", math.pi / 3.0).value
        )
        self.epsilon3 = float(self.declare_parameter("epsilon3", 0.25).value)
        self.rho1 = float(self.declare_parameter("rho1", 0.02).value)
        self.rho2 = float(self.declare_parameter("rho2", 0.02).value)
        self.omega_e = float(self.declare_parameter("omega_e", 0.5).value)

        # ESC gradient law and saturation from the ESC paper.
        self.gradient_gain = float(
            self.declare_parameter("gradient_gain", 1.0e-4).value
        )
        self.nu_max = float(self.declare_parameter("nu_max", 0.03).value)

        # By default use exactly one period Te = 2*pi/omega_e.
        # A positive user value overrides the automatically calculated period.
        requested_window = float(
            self.declare_parameter("estimation_window_sec", 0.0).value
        )
        self.estimation_window = (
            requested_window
            if requested_window > 0.0
            else 2.0 * math.pi / max(abs(self.omega_e), 1.0e-9)
        )
        self.polynomial_scale = float(
            self.declare_parameter("polynomial_scale", 0.10).value
        )
        self.min_samples = int(self.declare_parameter("min_samples", 12).value)
        self.max_fit_condition = float(
            self.declare_parameter("max_fit_condition", 1.0e7).value
        )

        # The sensor may not be exactly at the robot's odometry reference point.
        # Offset is expressed in the robot's body frame, in metres.
        self.sensor_offset_x = float(
            self.declare_parameter("sensor_offset_x", 0.0).value
        )
        self.sensor_offset_y = float(
            self.declare_parameter("sensor_offset_y", 0.0).value
        )

        # Conservative initial output limits. These are software safeguards,
        # not verified mechanical limits for every MTracker hardware revision.
        self.max_linear_speed = float(
            self.declare_parameter("max_linear_speed", 0.05).value
        )
        self.max_angular_speed = float(
            self.declare_parameter("max_angular_speed", 0.50).value
        )
        self.max_alpha_dot = float(
            self.declare_parameter("max_alpha_dot", 2.0).value
        )

        self.control_rate_hz = float(
            self.declare_parameter("control_rate_hz", 50.0).value
        )
        self.max_odom_age = float(
            self.declare_parameter("max_odom_age", 0.30).value
        )
        self.max_light_age = float(
            self.declare_parameter("max_light_age", 0.60).value
        )
        self.max_light_pair_delta = float(
            self.declare_parameter("max_light_pair_delta", 0.25).value
        )
        self.enable_motion = bool(
            self.declare_parameter("enable_motion", False).value
        )
        self.alpha = float(self.declare_parameter("alpha_initial", 0.0).value)
        self.alpha %= 2.0 * math.pi

        self._validate_parameters()

        # -------------------------- Internal state ---------------------------
        self._start_time = time.monotonic()
        self._last_loop_time = self._start_time
        self._last_log_time = 0.0
        self._last_warning_time = 0.0

        self._pose: Optional[np.ndarray] = None  # [x, y, yaw] in odom frame
        self._pose_rx_time: Optional[float] = None

        self._light_lux: Optional[float] = None
        self._light_rx_time: Optional[float] = None
        self._light_seq = 0
        self._last_sampled_light_seq = -1

        self._light_valid: Optional[bool] = None
        self._light_valid_rx_time: Optional[float] = None

        # Each entry is (monotonic_time, local_sensor_x, local_sensor_y, lux).
        self._samples: Deque[Tuple[float, float, float, float]] = deque()
        self._last_gradient: Optional[np.ndarray] = None
        self._last_fit_condition: Optional[float] = None

        # ------------------------------ ROS I/O ------------------------------
        self._cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self._odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 10
        )
        self._light_sub = self.create_subscription(
            Float32, self.light_topic, self._on_light, 10
        )
        self._light_valid_sub = self.create_subscription(
            Bool, self.light_valid_topic, self._on_light_valid, 10
        )

        self._timer = self.create_timer(
            1.0 / self.control_rate_hz, self._control_tick
        )

        self.get_logger().info(
            "MTracker ESC node started. "
            f"odom={self.odom_topic}, light={self.light_topic}, "
            f"valid={self.light_valid_topic}, cmd={self.cmd_topic}"
        )
        self.get_logger().info(
            f"Excitation: rho1={self.rho1:.3f} m, rho2={self.rho2:.3f} m, "
            f"omega_e={self.omega_e:.3f} rad/s, "
            f"window={self.estimation_window:.2f} s"
        )
        if not self.enable_motion:
            self.get_logger().warning(
                "Motion is DISABLED. After checking all topics and the "
                "workspace, enable with -p enable_motion:=true."
            )

    # ---------------------------------------------------------------------
    # Parameter and message handling
    # ---------------------------------------------------------------------
    def _validate_parameters(self) -> None:
        positive = {
            "omega_e": self.omega_e,
            "nu_max": self.nu_max,
            "estimation_window_sec": self.estimation_window,
            "polynomial_scale": self.polynomial_scale,
            "max_fit_condition": self.max_fit_condition,
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed,
            "max_alpha_dot": self.max_alpha_dot,
            "control_rate_hz": self.control_rate_hz,
            "max_odom_age": self.max_odom_age,
            "max_light_age": self.max_light_age,
            "max_light_pair_delta": self.max_light_pair_delta,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"Parameter '{name}' must be finite and > 0.")
        if self.rho1 < 0.0 or self.rho2 <= 0.0:
            raise ValueError("Use rho1 >= 0 and rho2 > 0.")
        if self.min_samples < 6:
            raise ValueError("min_samples must be at least 6 for a quadratic fit.")
        if self.gradient_gain <= 0.0:
            raise ValueError("gradient_gain must be > 0 for maximization.")

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """Extract planar yaw from a ROS quaternion."""
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(sin_yaw, cos_yaw)

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose = np.array(
            [
                float(position.x),
                float(position.y),
                self._yaw_from_quaternion(orientation),
            ],
            dtype=float,
        )
        if np.all(np.isfinite(pose)):
            self._pose = pose
            self._pose_rx_time = time.monotonic()

    def _on_light(self, msg: Float32) -> None:
        self._light_lux = float(msg.data)
        self._light_rx_time = time.monotonic()
        self._light_seq += 1

    def _on_light_valid(self, msg: Bool) -> None:
        self._light_valid = bool(msg.data)
        self._light_valid_rx_time = time.monotonic()

    # ---------------------------------------------------------------------
    # Estimator: local quadratic least-squares fit over one sliding window
    # ---------------------------------------------------------------------
    def _prune_samples(self, now_mono: float) -> None:
        cutoff = now_mono - self.estimation_window
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def _current_sensor_coordinates(self, f: np.ndarray) -> np.ndarray:
        """Return sensor position in the virtual frame's local XY coordinates.

        If the BH1750 is mounted at the base_link origin, both offsets are zero
        and this simply returns [f_x, f_y]. Otherwise rotate the body-frame
        sensor offset by f_theta and add it to the transverse translation.
        """
        c = math.cos(float(f[2]))
        s = math.sin(float(f[2]))
        offset_local = np.array(
            [
                c * self.sensor_offset_x - s * self.sensor_offset_y,
                s * self.sensor_offset_x + c * self.sensor_offset_y,
            ],
            dtype=float,
        )
        return np.asarray(f[:2], dtype=float) + offset_local

    def _try_add_latest_measurement(
        self, now_mono: float, f: np.ndarray
    ) -> None:
        """Add each new valid light measurement once, paired by receive time."""
        if self._light_seq == self._last_sampled_light_seq:
            return
        if self._light_rx_time is None or self._light_valid_rx_time is None:
            return

        pair_delta = abs(self._light_rx_time - self._light_valid_rx_time)
        if pair_delta > self.max_light_pair_delta:
            return  # Wait briefly for the corresponding Bool message.

        # We have now handled this Float32 sample, whether valid or invalid.
        self._last_sampled_light_seq = self._light_seq

        light_age = now_mono - self._light_rx_time
        valid_age = now_mono - self._light_valid_rx_time
        if light_age > self.max_light_age or valid_age > self.max_light_age:
            return
        if not self._light_valid:
            return
        if self._light_lux is None or not math.isfinite(self._light_lux):
            return
        if self._light_lux < 0.0:
            return

        sensor_xy = self._current_sensor_coordinates(f)
        self._samples.append(
            (
                self._light_rx_time,
                float(sensor_xy[0]),
                float(sensor_xy[1]),
                float(self._light_lux),
            )
        )
        self._prune_samples(now_mono)

    def _estimate_local_gradient(self) -> Optional[np.ndarray]:
        """Fit a 2nd-order polynomial and return gradient at local origin.

        mu_hat(x,y) = c0 + c1*xn + c2*yn + c3*xn^2 + c4*xn*yn + c5*yn^2
        xn=x/polynomial_scale, yn=y/polynomial_scale.

        Thus d(mu_hat)/dx at (0,0) = c1/polynomial_scale and similarly for y.
        """
        if len(self._samples) < self.min_samples:
            return None

        data = np.asarray(list(self._samples), dtype=float)
        x = data[:, 1] / self.polynomial_scale
        y = data[:, 2] / self.polynomial_scale
        lux = data[:, 3]

        design = np.column_stack(
            (
                np.ones_like(x),
                x,
                y,
                x * x,
                x * y,
                y * y,
            )
        )

        # SVD-based least squares avoids explicitly forming A.T @ A.
        singular_values = np.linalg.svd(design, compute_uv=False)
        if singular_values.size < 6 or singular_values[-1] <= 1.0e-12:
            self._last_fit_condition = math.inf
            return None

        condition = float(singular_values[0] / singular_values[-1])
        self._last_fit_condition = condition
        if not math.isfinite(condition) or condition > self.max_fit_condition:
            return None

        coefficients, _, rank, _ = np.linalg.lstsq(design, lux, rcond=None)
        if rank < 6 or not np.all(np.isfinite(coefficients)):
            return None

        gradient = np.array(
            [
                coefficients[1] / self.polynomial_scale,
                coefficients[2] / self.polynomial_scale,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(gradient)):
            return None
        return gradient

    # ---------------------------------------------------------------------
    # ESC and Transverse Function control
    # ---------------------------------------------------------------------
    def _virtual_input_from_gradient(
        self, local_gradient: Optional[np.ndarray]
    ) -> np.ndarray:
        """ESC law: nu = k * [dmu/df_x, dmu/df_y, 0]^T.

        This is the local-coordinate form of Eqs. (20)-(25) in the ESC paper.
        The transformation to the inertial gradient and back to the body/local
        input cancels through X(z)^T and X(z)^(-1), leaving the local gradient.
        """
        if local_gradient is None:
            # Keep the TF excitation running while collecting enough samples.
            return np.zeros(3, dtype=float)

        nu = np.array(
            [
                self.gradient_gain * float(local_gradient[0]),
                self.gradient_gain * float(local_gradient[1]),
                0.0,
            ],
            dtype=float,
        )

        # Eq. (26): infinity-norm scaling of the virtual input.
        norm_inf = float(np.linalg.norm(nu, ord=np.inf))
        scale = max(1.0, norm_inf / self.nu_max)
        return nu / scale

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _warn_throttled(self, message: str, period: float = 2.0) -> None:
        now_mono = time.monotonic()
        if now_mono - self._last_warning_time >= period:
            self.get_logger().warning(message)
            self._last_warning_time = now_mono

    def _control_tick(self) -> None:
        now_mono = time.monotonic()
        dt = now_mono - self._last_loop_time
        self._last_loop_time = now_mono
        if not math.isfinite(dt) or dt <= 0.0:
            self._publish_stop()
            return

        elapsed = now_mono - self._start_time

        # Safety is explicit: no movement until the user enables it.
        if not self.enable_motion:
            self._publish_stop()
            return

        # Stop if either odometry or the light sensor is unavailable/stale.
        if self._pose is None or self._pose_rx_time is None:
            self._publish_stop()
            self._warn_throttled("Waiting for /wheel/odom; command set to zero.")
            return
        if now_mono - self._pose_rx_time > self.max_odom_age:
            self._publish_stop()
            self._warn_throttled("Odometry is stale; command set to zero.")
            return
        if (
            self._light_rx_time is None
            or self._light_valid_rx_time is None
            or self._light_valid is not True
            or self._light_lux is None
            or not math.isfinite(self._light_lux)
            or now_mono - self._light_rx_time > self.max_light_age
            or now_mono - self._light_valid_rx_time > self.max_light_age
            or abs(self._light_rx_time - self._light_valid_rx_time)
            > self.max_light_pair_delta
        ):
            self._publish_stop()
            self._warn_throttled(
                "No fresh valid light measurement; command set to zero. "
                "Check BH1750 firmware and LIGHT_INTENSITY_BH1750 build option."
            )
            return

        try:
            # Compute f and all TF terms at the current alpha and elapsed time.
            components = calculate_components(
                epsilon1=self.epsilon1,
                epsilon2=self.epsilon2,
                epsilon3=self.epsilon3,
                rho1=self.rho1,
                rho2=self.rho2,
                omega_e=self.omega_e,
                alpha=self.alpha,
                t=elapsed,
            )
            f = np.asarray(components["f"], dtype=float)
            f_inverse = np.asarray(components["f_inverse"], dtype=float)

            # g = z*f, hence z = g*f^(-1). Odometry is used to reconstruct
            # and log the virtual pose. The local polynomial uses f_xy.
            virtual_pose = se2_compose(self._pose, f_inverse)

            # Process the most recent sensor message once and update the window.
            self._try_add_latest_measurement(now_mono, f)
            self._prune_samples(now_mono)

            local_gradient = self._estimate_local_gradient()
            self._last_gradient = (
                None if local_gradient is None else local_gradient.copy()
            )
            nu = self._virtual_input_from_gradient(local_gradient)

            # Geometric controller from controller.py: [vx, omega, alpha_dot].
            u_bar = extended_control(
                nu=nu,
                epsilon1=self.epsilon1,
                epsilon2=self.epsilon2,
                epsilon3=self.epsilon3,
                rho1=self.rho1,
                rho2=self.rho2,
                omega_e=self.omega_e,
                alpha=self.alpha,
                t=elapsed,
            )
            vx, omega = physical_control(u_bar)
            alpha_dot = float(u_bar[2])

            # Strict output guard. Do NOT silently clip only vx/omega: that
            # would invalidate the exact TF decoupling relation. The command is
            # stopped and the user can lower excitation parameters or adjust
            # limits after inspecting the reported values.
            violations = []
            if abs(vx) > self.max_linear_speed:
                violations.append(
                    f"|vx|={abs(vx):.3f}>{self.max_linear_speed:.3f} m/s"
                )
            if abs(omega) > self.max_angular_speed:
                violations.append(
                    f"|omega|={abs(omega):.3f}>{self.max_angular_speed:.3f} rad/s"
                )
            if abs(alpha_dot) > self.max_alpha_dot:
                violations.append(
                    f"|alpha_dot|={abs(alpha_dot):.3f}>{self.max_alpha_dot:.3f} rad/s"
                )
            if violations:
                self._publish_stop()
                self._warn_throttled(
                    "TF output exceeded safety limit; robot stopped: "
                    + "; ".join(violations)
                    + ". Tune rho1/rho2/omega_e/epsilon or the configured limits."
                )
                return

            command = Twist()
            command.linear.x = float(vx)
            command.angular.z = float(omega)
            self._cmd_pub.publish(command)

            # Integrate the internal transverse-function phase only after a
            # valid command has been produced and published.
            self.alpha = integrate_alpha(self.alpha, alpha_dot, dt)

            # Compact diagnostics at 1 Hz.
            if elapsed - self._last_log_time >= 1.0:
                self._last_log_time = elapsed
                if local_gradient is None:
                    grad_text = "gradient=not_ready"
                else:
                    grad_text = (
                        f"grad_local=[{local_gradient[0]:.3f}, "
                        f"{local_gradient[1]:.3f}] lx/m"
                    )
                fit_text = (
                    "n/a"
                    if self._last_fit_condition is None
                    else f"{self._last_fit_condition:.2e}"
                )
                self.get_logger().info(
                    f"lux={self._light_lux:.2f}; samples={len(self._samples)}; "
                    f"{grad_text}; fit_cond={fit_text}; "
                    f"nu=[{nu[0]:.4f}, {nu[1]:.4f}, {nu[2]:.4f}]; "
                    f"u=[vx={vx:.4f}, omega={omega:.4f}, "
                    f"alpha_dot={alpha_dot:.4f}]; "
                    f"z=[{virtual_pose[0]:.3f}, {virtual_pose[1]:.3f}, "
                    f"{virtual_pose[2]:.3f}]"
                )

        except (ValueError, np.linalg.LinAlgError, ArithmeticError) as exc:
            self._publish_stop()
            self._warn_throttled(f"ESC calculation failed; command set to zero: {exc}")

    def stop(self) -> None:
        """Publish a final zero-velocity command."""
        try:
            self._publish_stop()
        except Exception:  # ROS context may already be shutting down.
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MTrackerExtremumSeekingNode] = None
    try:
        node = MTrackerExtremumSeekingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
