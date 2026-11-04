#!/usr/bin/env python3
"""Extremum Seeking Controller for a unicycle-like robot."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Deque, Optional, Tuple

import numpy as np


# ============================================================
# SE(2) OPERATIONS
# ============================================================

def _rotation(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def se2_compose(g, h):
    g = np.asarray(g, dtype=float).reshape(3)
    h = np.asarray(h, dtype=float).reshape(3)
    xy = g[:2] + _rotation(g[2]) @ h[:2]
    return np.array([xy[0], xy[1], g[2] + h[2]])


def se2_inverse(g):
    g = np.asarray(g, dtype=float).reshape(3)
    xy = -_rotation(-g[2]) @ g[:2]
    return np.array([xy[0], xy[1], -g[2]])


def se2_adjoint(g):
    x, y, theta = np.asarray(g, dtype=float).reshape(3)
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, y],
        [s,  c, -x],
        [0., 0., 1.]
    ])


def left_trivialized_vector(g, g_dot):
    theta = float(np.asarray(g).reshape(3)[2])
    derivative = np.asarray(g_dot).reshape(3)
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        c * derivative[0] + s * derivative[1],
        -s * derivative[0] + c * derivative[1],
        derivative[2]
    ])


# ============================================================
# GRADIENT ESTIMATOR – EQUATIONS (22)-(24)
# ============================================================

@dataclass
class GradientEstimate:
    gradient: np.ndarray
    sample_count: int
    condition_number: float
    residual_rms: float


class SlidingWindowQuadraticEstimator:

    def __init__(
        self,
        window_seconds,
        min_samples=20,
        max_condition_number=1e8
    ):
        if window_seconds <= 0:
            raise ValueError("window_seconds must be positive")

        self.window_seconds = float(window_seconds)
        self.min_samples = int(min_samples)
        self.max_condition_number = float(max_condition_number)
        self._samples: Deque[Tuple[float, float, float, float]] = deque()

    def add_sample(self, t, x, y, intensity):
        values = tuple(map(float, (t, x, y, intensity)))

        if not all(math.isfinite(v) for v in values):
            return

        self._samples.append(values)
        self._remove_old_samples(t)

    def _remove_old_samples(self, now):
        cutoff = now - self.window_seconds

        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()

    def estimate(self, now):
        self._remove_old_samples(now)

        if len(self._samples) < self.min_samples:
            return None

        data = np.asarray(self._samples, dtype=float)
        times = data[:, 0]

        if times[-1] - times[0] < 0.90 * self.window_seconds:
            return None

        x = data[:, 1]
        y = data[:, 2]
        mu = data[:, 3]

        scale = max(float(np.sqrt(np.mean(x*x + y*y))), 1e-3)

        xn = x / scale
        yn = y / scale

        # Local quadratic model:
        # mu = b0 + b1*x + b2*y
        #      + 0.5*b3*x² + b4*x*y + 0.5*b5*y²

        design = np.column_stack((
            np.ones_like(xn),
            xn,
            yn,
            0.5 * xn**2,
            xn * yn,
            0.5 * yn**2
        ))

        try:
            condition = float(np.linalg.cond(design))

            if not math.isfinite(condition):
                return None

            if condition > self.max_condition_number:
                return None

            coefficients, _, rank, _ = np.linalg.lstsq(
                design, mu, rcond=None
            )

        except np.linalg.LinAlgError:
            return None

        if rank < 6 or not np.all(np.isfinite(coefficients)):
            return None

        gradient = coefficients[1:3] / scale

        residual = design @ coefficients - mu
        residual_rms = float(np.sqrt(np.mean(residual**2)))

        return GradientEstimate(
            gradient=gradient,
            sample_count=len(self._samples),
            condition_number=condition,
            residual_rms=residual_rms
        )

    @property
    def sample_count(self):
        return len(self._samples)


# ============================================================
# CONTROLLER CONFIGURATION
# ============================================================

@dataclass
class ControllerConfig:

    # Transverse function – equation (15)
    eps1: float = 0.1
    eps2: float = math.pi / 3.0
    eps3: float = 0.25

    # Excitation – equation (16)
    rho1: float = 0.3
    rho2: float = 0.3
    omega_e: float = 0.5

    # Virtual controller – equations (25)-(26)
    k: float = 0.03
    nu_max: float = 0.05

    # Physical robot limits
    max_linear_speed: float = 0.30
    max_angular_speed: float = 4.0

    # Estimator
    min_estimator_samples: int = 20
    max_estimator_condition: float = 1e8

    def validate(self):

        if self.eps1 <= 0 or self.eps2 <= 0:
            raise ValueError("eps1 and eps2 must be positive")

        if self.rho1 < 0 or self.rho2 <= 0:
            raise ValueError("rho1 >= 0 and rho2 > 0 required")

        if self.omega_e <= 0:
            raise ValueError("omega_e must be positive")

        if self.k <= 0 or self.nu_max <= 0:
            raise ValueError("k and nu_max must be positive")

        if self.max_linear_speed <= 0 or self.max_angular_speed <= 0:
            raise ValueError("Robot speed limits must be positive")

    @property
    def excitation_period(self):
        return 2.0 * math.pi / self.omega_e


@dataclass
class ControlOutput:
    linear_velocity: float
    angular_velocity: float
    alpha_dot: float
    alpha: float
    transverse_function: np.ndarray
    gradient: Optional[np.ndarray]
    virtual_velocity: np.ndarray
    estimator_samples: int
    estimator_condition: Optional[float]
    robot_command_saturated: bool
# ============================================================
# EXTREMUM SEEKING CONTROLLER
# ============================================================

class ExtremumSeekingController:

    def __init__(self, config=None):

        self.config = config or ControllerConfig()
        self.config.validate()

        self.alpha = 0.0

        self.estimator = SlidingWindowQuadraticEstimator(
            window_seconds=self.config.excitation_period,
            min_samples=self.config.min_estimator_samples,
            max_condition_number=self.config.max_estimator_condition
        )

    # --------------------------------------------------------
    # Transverse function – equations (14)-(16)
    # --------------------------------------------------------

    def transverse_function(self, alpha, t):

        c = self.config
        wt = c.omega_e * t

        f_bar = np.array([
            c.eps1 * math.sin(alpha),
            c.eps1 * c.eps2 * c.eps3 * math.sin(2.0 * alpha),
            c.eps2 * math.cos(alpha)
        ])

        h = np.array([
            c.rho1 * math.cos(wt),
            c.rho2 * math.sin(wt),
            0.0
        ])

        f = se2_compose(h, f_bar)

        return f, h, f_bar

    def _transverse_derivative(self, alpha):

        c = self.config

        return np.array([
            c.eps1 * math.cos(alpha),
            2.0 * c.eps1 * c.eps2 * c.eps3
            * math.cos(2.0 * alpha),
            -c.eps2 * math.sin(alpha)
        ])

    # --------------------------------------------------------
    # Extended control – equation (19)
    # --------------------------------------------------------

    def _extended_control(self, alpha, t, virtual_velocity):

        c = self.config

        f, h, f_bar = self.transverse_function(alpha, t)

        df_dalpha = self._transverse_derivative(alpha)

        # A_alpha = X(f)^(-1) * df/dalpha
        a_alpha = left_trivialized_vector(f, df_dalpha)

        # Unicycle input matrix
        C = np.array([
            [1.0, 0.0],
            [0.0, 0.0],
            [0.0, 1.0]
        ])

        C_bar = np.column_stack((C, -a_alpha))

        # h_dot
        wt = c.omega_e * t

        h_dot = np.array([
            -c.rho1 * c.omega_e * math.sin(wt),
            c.rho2 * c.omega_e * math.cos(wt),
            0.0
        ])

        # w_h = X(h)^(-1) * h_dot
        w_h = left_trivialized_vector(h, h_dot)

        # Equation (19)
        rhs = se2_adjoint(se2_inverse(f)) @ (
            np.asarray(virtual_velocity).reshape(3)
            + se2_adjoint(h) @ w_h
        )

        try:
            if np.linalg.cond(C_bar) > 1e8:
                raise np.linalg.LinAlgError(
                    "C_bar is numerically singular"
                )

            extended_input = np.linalg.solve(C_bar, rhs)

        except np.linalg.LinAlgError as exc:
            raise RuntimeError(
                f"Transverse-function inversion failed: {exc}"
            ) from exc

        vx, omega, alpha_dot = extended_input

        return float(vx), float(omega), float(alpha_dot)

    # --------------------------------------------------------
    # Main controller update
    # --------------------------------------------------------

    def step(
        self,
        t,
        dt,
        intensity=None,
        new_measurement=False
    ):

        if dt <= 0 or not math.isfinite(dt):
            raise ValueError("dt must be finite and positive")

        if not math.isfinite(t):
            raise ValueError("t must be finite")

        f, _, _ = self.transverse_function(self.alpha, t)

        # Store only genuinely new sensor readings.
        if (
            new_measurement
            and intensity is not None
            and math.isfinite(float(intensity))
        ):
            self.estimator.add_sample(
                t,
                f[0],
                f[1],
                float(intensity)
            )

        estimate = self.estimator.estimate(t)

        gradient = (
            None if estimate is None
            else estimate.gradient.copy()
        )

        # Equation (25):
        # nu = k * gradient^T
        virtual_velocity = np.zeros(3)

        if gradient is not None:
            virtual_velocity[:2] = self.config.k * gradient

        # Equation (26): infinity-norm saturation.
        norm_inf = float(np.max(np.abs(virtual_velocity)))

        scale = max(
            1.0,
            norm_inf / self.config.nu_max
        )

        virtual_velocity /= scale

        # Convert virtual velocity into unicycle commands.
        try:
            vx, omega, alpha_dot = self._extended_control(
                self.alpha,
                t,
                virtual_velocity
            )

        except RuntimeError:
            vx = 0.0
            omega = 0.0
            alpha_dot = 0.0

        # Additional physical velocity limits.
        raw_vx = vx
        raw_omega = omega

        vx = float(np.clip(
            vx,
            -self.config.max_linear_speed,
            self.config.max_linear_speed
        ))

        omega = float(np.clip(
            omega,
            -self.config.max_angular_speed,
            self.config.max_angular_speed
        ))

        saturated = (
            not math.isclose(vx, raw_vx, abs_tol=1e-9)
            or not math.isclose(omega, raw_omega, abs_tol=1e-9)
        )

        if not all(math.isfinite(v) for v in (vx, omega, alpha_dot)):
            vx = omega = alpha_dot = 0.0
            saturated = True

        # Integrate the internal transverse-function coordinate.
        self.alpha = (
            self.alpha + alpha_dot * dt
        ) % (2.0 * math.pi)

        return ControlOutput(
            linear_velocity=vx,
            angular_velocity=omega,
            alpha_dot=alpha_dot,
            alpha=self.alpha,
            transverse_function=f,
            gradient=gradient,
            virtual_velocity=virtual_velocity.copy(),
            estimator_samples=self.estimator.sample_count,
            estimator_condition=(
                None if estimate is None
                else estimate.condition_number
            ),
            robot_command_saturated=saturated
        )

# ============================================================
# ROS 2 NODE
# ============================================================

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import (
        Float32, Float64,
        Int16, Int32, UInt16, UInt32
    )

    try:
        from sensor_msgs.msg import Illuminance
    except ImportError:
        Illuminance = None

    ROS_AVAILABLE = True

except ImportError:
    rclpy = None
    Node = object
    Twist = None
    Float32 = None
    Float64 = Int16 = Int32 = UInt16 = UInt32 = None
    Illuminance = None
    ROS_AVAILABLE = False


if ROS_AVAILABLE:

    class LightExtremumSeekingNode(Node):

        def __init__(self):

            super().__init__(
                "light_extremum_seeking_controller"
            )

            # ROS parameters
            self.declare_parameter(
                "light_topic", "/light_intensity"
            )

            self.declare_parameter(
                "light_message_type", "Float32"
            )

            self.declare_parameter(
                "cmd_vel_topic", "/cmd_vel"
            )

            self.declare_parameter(
                "control_rate", 10.0
            )

            self.declare_parameter(
                "sensor_timeout", 1.0
            )

            defaults = ControllerConfig()

            parameter_names = [
                "eps1", "eps2", "eps3",
                "rho1", "rho2", "omega_e",
                "k", "nu_max",
                "max_linear_speed",
                "max_angular_speed",
                "min_estimator_samples",
                "max_estimator_condition"
            ]

            for name in parameter_names:
                self.declare_parameter(
                    name, getattr(defaults, name)
                )

            values = {
                name: self.get_parameter(name).value
                for name in parameter_names
            }

            values["min_estimator_samples"] = int(
                values["min_estimator_samples"]
            )

            self.controller = ExtremumSeekingController(
                ControllerConfig(**values)
            )

            light_topic = str(
                self.get_parameter("light_topic").value
            )

            cmd_vel_topic = str(
                self.get_parameter("cmd_vel_topic").value
            )

            control_rate = float(
                self.get_parameter("control_rate").value
            )

            self.sensor_timeout = float(
                self.get_parameter("sensor_timeout").value
            )

            if control_rate <= 0 or self.sensor_timeout <= 0:
                raise ValueError(
                    "control_rate and sensor_timeout must be positive"
                )

            # Supported sensor message types
            message_types = {
                "Float32": Float32,
                "Float64": Float64,
                "Int16": Int16,
                "Int32": Int32,
                "UInt16": UInt16,
                "UInt32": UInt32
            }

            if Illuminance is not None:
                message_types["Illuminance"] = Illuminance

            message_type = str(
                self.get_parameter(
                    "light_message_type"
                ).value
            )

            if message_type not in message_types:
                raise ValueError(
                    f"Unsupported message type: {message_type}"
                )

            self.publisher = self.create_publisher(
                Twist,
                cmd_vel_topic,
                10
            )

            self.subscription = self.create_subscription(
                message_types[message_type],
                light_topic,
                self._light_callback,
                10
            )

            self.latest_intensity = None
            self.last_sensor_time = None
            self.new_measurement = False

            self.start_time = time.monotonic()
            self.last_control_time = self.start_time
            self.last_log_time = self.start_time

            self.timer = self.create_timer(
                1.0 / control_rate,
                self._control_callback
            )

            self.get_logger().info(
                f"ESC started: {light_topic} -> {cmd_vel_topic}"
            )

            self.get_logger().info(
                f"Excitation period: "
                f"{self.controller.config.excitation_period:.2f} s"
            )

        # ----------------------------------------------------
        # Light sensor callback
        # ----------------------------------------------------

        def _light_callback(self, msg):

            raw_value = None

            for field in (
                "data",
                "illuminance",
                "intensity",
                "value"
            ):

                if hasattr(msg, field):
                    raw_value = getattr(msg, field)
                    break

            if raw_value is None:
                self.get_logger().warning(
                    "Light message has no supported intensity field"
                )
                return

            value = float(raw_value)

            if not math.isfinite(value):
                return

            self.latest_intensity = value
            self.last_sensor_time = time.monotonic()
            self.new_measurement = True

        # ----------------------------------------------------
        # Stop robot
        # ----------------------------------------------------

        def _publish_stop(self):

            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.publisher.publish(msg)

        # ----------------------------------------------------
        # Main control loop
        # ----------------------------------------------------

        def _control_callback(self):

            now = time.monotonic()

            dt = now - self.last_control_time
            self.last_control_time = now

            elapsed = now - self.start_time

            # Stop when sensor data is missing or outdated.
            if (
                self.latest_intensity is None
                or self.last_sensor_time is None
                or now - self.last_sensor_time > self.sensor_timeout
            ):

                self._publish_stop()
                self.new_measurement = False
                return

            output = self.controller.step(
                t=elapsed,
                dt=max(dt, 1e-4),
                intensity=self.latest_intensity,
                new_measurement=self.new_measurement
            )

            self.new_measurement = False

            # Publish velocity command.
            msg = Twist()

            msg.linear.x = output.linear_velocity
            msg.angular.z = output.angular_velocity

            self.publisher.publish(msg)

            # Diagnostic log every 3 seconds.
            if now - self.last_log_time >= 3.0:

                if output.gradient is None:
                    gradient_text = "collecting samples"
                else:
                    gradient_text = (
                        f"[{output.gradient[0]:.4f}, "
                        f"{output.gradient[1]:.4f}]"
                    )

                self.get_logger().info(
                    f"I={self.latest_intensity:.3f}, "
                    f"gradient={gradient_text}, "
                    f"v={output.linear_velocity:.3f}, "
                    f"omega={output.angular_velocity:.3f}, "
                    f"samples={output.estimator_samples}"
                )

                self.last_log_time = now

        def destroy_node(self):

            try:
                self._publish_stop()
            except Exception:
                pass

            return super().destroy_node()

else:

    class LightExtremumSeekingNode:

        def __init__(self):
            raise RuntimeError(
                "ROS 2 modules unavailable. "
                "Source the ROS 2 setup.bash first."
            )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    if not ROS_AVAILABLE:
        raise SystemExit(
            "ROS 2 is not available. "
            "Source /opt/ros/<distro>/setup.bash first."
        )

    rclpy.init(args=args)

    node = LightExtremumSeekingNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
