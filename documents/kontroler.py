"""Transverse-function based ESC controller for a unicycle-like robot.

The SE(2) convention follows "Extremum Seeking Control of a Unicycle-like
Robot Using the Transverse Function Approach" (2026):
    g = [x, y, theta]^T,
    g1 * g2 = g1 + T(theta_1) @ g2,
    X(g) = T(theta).

The adjoint formula in Eq. (54) of "Waypoint Following for Differentially
Driven Wheeled Robots with Limited Velocity Perturbations" is expressed in
a different Lie-algebra basis.  The constant basis transformation below
converts it to the X(g)=T(theta) basis used here.
"""

from __future__ import annotations

import numpy as np


# Kinematics of the unicycle: g_dot = X(g) C u, u = [v_x, omega]^T.
# C is 3x2 (not 2x3): its columns select forward and angular velocity.
C = np.array(
    [
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, 1.0],
    ],
    dtype=float,
)

# The 2017 waypoint-following paper uses X_WP(g) = T(theta) @ S.
# S = X_WP(e), so Ad_WP(g) = S^{-1} Ad_SE2(g) S.
# Since S is orthogonal, S^{-1} = S.T.
_S_BASIS = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def _as_vec3(value: np.ndarray | list | tuple, name: str) -> np.ndarray:
    """Convert a 3-element input to a finite float vector."""
    result = np.asarray(value, dtype=float).reshape(-1)
    if result.shape != (3,):
        raise ValueError(f"{name} must contain exactly 3 elements [x, y, theta].")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain only finite values.")
    return result


def T(theta: float) -> np.ndarray:
    """SE(2) left-translation matrix X(g)=T(theta)."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def se2_compose(g1: np.ndarray, g2: np.ndarray) -> np.ndarray:
    """Group product g1*g2 for SE(2), using [x, y, theta] coordinates."""
    g1 = _as_vec3(g1, "g1")
    g2 = _as_vec3(g2, "g2")
    return g1 + T(g1[2]) @ g2


def se2_inverse(g: np.ndarray) -> np.ndarray:
    """Group inverse in SE(2): g^{-1} = -T(theta)^T g."""
    g = _as_vec3(g, "g")
    return -(T(g[2]).T @ g)


def h(p1: float, p2: float, omega: float, t: float) -> np.ndarray:
    """Time-varying translating element h(t), Eq. (16) of the ESC paper."""
    return np.array(
        [
            p1 * np.cos(omega * t),
            p2 * np.sin(omega * t),
            0.0,
        ],
        dtype=float,
    )


def dh(p1: float, p2: float, omega: float, t: float) -> np.ndarray:
    """Time derivative of h(t)."""
    return np.array(
        [
            -omega * p1 * np.sin(omega * t),
            omega * p2 * np.cos(omega * t),
            0.0,
        ],
        dtype=float,
    )


def f_kreska(
    epsilon1: float,
    epsilon2: float,
    epsilon3: float,
    alpha: float,
) -> np.ndarray:
    """Base transverse function f_bar(alpha), Eq. (15) of the ESC paper."""
    return np.array(
        [
            epsilon1 * np.sin(alpha),
            epsilon1 * epsilon2 * epsilon3 * np.sin(2.0 * alpha),
            epsilon2 * np.cos(alpha),
        ],
        dtype=float,
    )


def df_kreska_dalpha(
    epsilon1: float,
    epsilon2: float,
    epsilon3: float,
    alpha: float,
) -> np.ndarray:
    """Analytic derivative d f_bar / d alpha."""
    return np.array(
        [
            epsilon1 * np.cos(alpha),
            2.0 * epsilon1 * epsilon2 * epsilon3 * np.cos(2.0 * alpha),
            -epsilon2 * np.sin(alpha),
        ],
        dtype=float,
    )


def Ad_X_waypoint(g: np.ndarray) -> np.ndarray:
    """Adjoint matrix in the basis used in Eq. (54) of the waypoint paper.

    Eq. (54):
        [[ cos(theta),  y, sin(theta)],
         [          0,  1,          0],
         [-sin(theta),  x, cos(theta)]].

    This is NOT directly the basis used by the 2026 ESC paper.
    """
    x, y, theta = _as_vec3(g, "g")
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array(
        [
            [c, y, s],
            [0.0, 1.0, 0.0],
            [-s, x, c],
        ],
        dtype=float,
    )


def Ad_X(g: np.ndarray) -> np.ndarray:
    """Adjoint representation in the ESC basis X(g)=T(theta).

    The waypoint paper's basis satisfies X_WP(g)=T(theta)@S.  Therefore
        Ad_WP(g) = S.T @ Ad_ESC(g) @ S,
    and the required conversion is
        Ad_ESC(g) = S @ Ad_WP(g) @ S.T.

    The resulting explicit matrix is
        [[ cos(theta), -sin(theta),  y],
         [ sin(theta),  cos(theta), -x],
         [          0,           0,  1]].
    """
    return _S_BASIS @ Ad_X_waypoint(g) @ _S_BASIS.T


def calculate_components(
    epsilon1: float,
    epsilon2: float,
    epsilon3: float,
    rho1: float,
    rho2: float,
    omega_e: float,
    alpha: float,
    t: float,
) -> dict[str, np.ndarray | float]:
    """Calculate f, both adjoint matrices, A_alpha and C_bar.

    A_alpha = X(f)^(-1) * partial(f)/partial(alpha)
    C_bar   = [ C  -A_alpha ].

    Returns intermediate values to make the implementation auditable.
    """
    h_value = h(rho1, rho2, omega_e, t)
    h_dot = dh(rho1, rho2, omega_e, t)
    f_bar = f_kreska(epsilon1, epsilon2, epsilon3, alpha)

    # The paper defines f(alpha,t) = h(t) * f_bar(alpha), using the SE(2)
    # group product (not element-wise multiplication).
    f_value = se2_compose(h_value, f_bar)
    f_inverse = se2_inverse(f_value)

    # Derivative of f with respect to alpha.  h is independent of alpha.
    df_bar = df_kreska_dalpha(epsilon1, epsilon2, epsilon3, alpha)
    df_dalpha = T(h_value[2]) @ df_bar
    A_alpha = np.linalg.solve(T(f_value[2]), df_dalpha)

    # Extended decoupling matrix C_bar = [C  -A_alpha], Eq. (12)/(13).
    C_bar = np.column_stack((C, -A_alpha))

    # h_dot expressed in the Lie-algebra basis at h:
    # w_h = X(h)^(-1) h_dot.  For this specific h, h_theta=0, hence
    # X(h)=I and w_h=h_dot; solve() keeps the formula general.
    w_h = np.linalg.solve(T(h_value[2]), h_dot)

    Ad_f_inverse = Ad_X(f_inverse)
    Ad_h = Ad_X(h_value)

    # Eq. (19): Ad_X(f^-1) * (nu + Ad_X(h) * X(h)^-1 * h_dot).
    # nu is inserted later, therefore retain the time-varying term here.
    time_term = Ad_f_inverse @ (Ad_h @ w_h)

    # Eq. (17) gives the equivalent A_t = Ad_X(f_bar^-1) * w_h.
    # Keeping this independently computed form is useful for checking Eq. (19).
    f_bar_inverse = se2_inverse(f_bar)
    A_t = Ad_X(f_bar_inverse) @ w_h

    return {
        "h": h_value,
        "h_dot": h_dot,
        "w_h": w_h,
        "f_bar": f_bar,
        "f": f_value,
        "f_inverse": f_inverse,
        "Ad_X_f_inverse": Ad_f_inverse,
        "Ad_X_h": Ad_h,
        "A_alpha": A_alpha,
        "C_bar": C_bar,
        "A_t": A_t,
        "time_term": time_term,
        "C_bar_condition": float(np.linalg.cond(C_bar)),
    }


def extended_control(
    nu: np.ndarray,
    epsilon1: float,
    epsilon2: float,
    epsilon3: float,
    rho1: float,
    rho2: float,
    omega_e: float,
    alpha: float,
    t: float,
    *,
    max_condition: float = 1.0e10,
    check_equivalent_forms: bool = True,
) -> np.ndarray:
    """Compute the extended input u_bar=[v_x, omega, alpha_dot]^T.

    Implements Eq. (19), equivalent to Eq. (13):
        u_bar = C_bar^(-1) Ad_X(f^(-1))
                * (nu + Ad_X(h) X(h)^(-1) h_dot).

    `nu` is the desired virtual-system velocity [nu_x, nu_y, nu_theta].
    The physical robot command is u=[u_bar[0], u_bar[1]], while
    alpha_dot=u_bar[2] is the additional virtual input.
    """
    nu = _as_vec3(nu, "nu")
    terms = calculate_components(
        epsilon1=epsilon1,
        epsilon2=epsilon2,
        epsilon3=epsilon3,
        rho1=rho1,
        rho2=rho2,
        omega_e=omega_e,
        alpha=alpha,
        t=t,
    )

    condition = float(terms["C_bar_condition"])
    if not np.isfinite(condition) or condition > max_condition:
        raise np.linalg.LinAlgError(
            "C_bar is singular or ill-conditioned "
            f"(cond={condition:.3e}). Check the transverse-function "
            "parameters and alpha; the transversality condition may fail."
        )

    Ad_f_inverse = np.asarray(terms["Ad_X_f_inverse"])
    Ad_h = np.asarray(terms["Ad_X_h"])
    w_h = np.asarray(terms["w_h"])
    C_bar = np.asarray(terms["C_bar"])

    # Eq. (19), with the exact order of matrix multiplication.
    rhs_eq19 = Ad_f_inverse @ (nu + Ad_h @ w_h)

    if check_equivalent_forms:
        # Eq. (13): Ad_X(f^-1) nu + A_t.
        rhs_eq13 = Ad_f_inverse @ nu + np.asarray(terms["A_t"])
        if not np.allclose(rhs_eq19, rhs_eq13, rtol=1.0e-9, atol=1.0e-11):
            raise ArithmeticError(
                "The Eq. (13) and Eq. (19) right-hand sides do not agree. "
                "Check the SE(2) group convention and adjoint basis conversion."
            )

    # Solve C_bar u_bar = rhs; do not explicitly calculate C_bar^{-1}.
    return np.linalg.solve(C_bar, rhs_eq19)


def physical_control(u_bar: np.ndarray) -> np.ndarray:
    """Extract physical unicycle input u=[v_x, omega] from u_bar."""
    u_bar = _as_vec3(u_bar, "u_bar")
    return u_bar[:2].copy()


if __name__ == "__main__":
    # Example values only. Replace them with the parameters selected for
    # the experiment/simulation and provide the virtual input nu online.
    epsilon1 = 0.1
    epsilon2 = np.pi / 3.0
    epsilon3 = 0.25
    rho1 = 0.2
    rho2 = 0.4
    omega_e = 2.0 * np.pi
    alpha = 0.5
    t = 0.0
    nu = np.array([0.1, 0.0, 0.0])

    u_bar = extended_control(
        nu,
        epsilon1,
        epsilon2,
        epsilon3,
        rho1,
        rho2,
        omega_e,
        alpha,
        t,
    )
    print("u_bar = [v_x, omega, alpha_dot] =", u_bar)
    print("u = [v_x, omega] =", physical_control(u_bar))
