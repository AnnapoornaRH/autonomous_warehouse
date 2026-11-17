"""Forward / inverse kinematics of the 4-DOF warehouse arm (pure python, no ROS).

Joint conventions (match mobile_manipulator.urdf.xacro):
    q1  turret yaw about +z (positive = counter-clockwise seen from above)
    q2  shoulder pitch about +y (positive = tilt the upper arm *forward*, 0 = straight up)
    q3  elbow pitch about +y     (positive = fold the forearm further forward/down)
    q4  wrist pitch about +y     (positive = tilt the hand further forward/down)

All links extend along their local +z axis.  Measured from the vertical, link angles are
    phi1 = q2,  phi2 = q2 + q3,  phi3 = q2 + q3 + q4
and the planar position of the hand reference point in the arm plane is
    r = L1 sin(phi1) + L2 sin(phi2) + L3 sin(phi3)
    z = L1 cos(phi1) + L2 cos(phi2) + L3 cos(phi3)
A hand pointing horizontally forward has phi3 = pi/2.

Frame: "arm frame" has its origin at the shoulder axis (centre of arm_joint2) and x forward.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

# Geometry (metres) -- keep in sync with the xacro
BASE_TO_SHOULDER = (0.05, 0.0, 0.21 + 0.04 + 0.06)   # arm_base_link + joint1 + link1 offset in base_link
L1 = 0.26
L2 = 0.26
L3 = 0.05 + 0.03 + 0.035 + 0.0   # wrist->gripper origin (0.05) + finger origin (0.03) + half finger (0.035)
GRIP_REACH = L3                                   # distance wrist axis -> grasp centre

Q_LIMITS = {
    "q1": (-1.8, 1.8),
    "q2": (-1.5, 1.8),
    "q3": (-0.5, 2.9),
    "q4": (-2.2, 2.2),
}

# Named configurations (q1, q2, q3, q4)
STOW = (0.0, -1.3, 2.85, -0.30)     # folded over the deck: the carried item stays inside the chassis footprint;
                                     # hand tilted ~18 deg UP so the item is pressed against the palm
READY = (0.0, 0.55, 1.50, -0.45)    # raised in front of the robot, gripper roughly level


def forward(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """Grasp-centre position in the *base_link* frame."""
    q1, q2, q3, q4 = q
    p1 = q2
    p2 = q2 + q3
    p3 = q2 + q3 + q4
    r = L1 * math.sin(p1) + L2 * math.sin(p2) + L3 * math.sin(p3)
    z = L1 * math.cos(p1) + L2 * math.cos(p2) + L3 * math.cos(p3)
    x = BASE_TO_SHOULDER[0] + r * math.cos(q1)
    y = BASE_TO_SHOULDER[1] + r * math.sin(q1)
    return (x, y, BASE_TO_SHOULDER[2] + z)


def link_points(q: Tuple[float, float, float, float]):
    """Elbow, wrist and grasp points in base_link (for clearance checks / debugging)."""
    q1, q2, q3, q4 = q
    p1, p2, p3 = q2, q2 + q3, q2 + q3 + q4
    pts = []
    r = z = 0.0
    for L, p in ((L1, p1), (L2, p2), (L3, p3)):
        r += L * math.sin(p)
        z += L * math.cos(p)
        pts.append((BASE_TO_SHOULDER[0] + r * math.cos(q1),
                    BASE_TO_SHOULDER[1] + r * math.sin(q1),
                    BASE_TO_SHOULDER[2] + z))
    return pts


def inverse(target_xyz: Tuple[float, float, float], pitch: float = math.pi / 2.0
            ) -> Optional[Tuple[float, float, float, float]]:
    """Solve (q1..q4) so that the grasp centre is at ``target_xyz`` (base_link frame) with the
    hand direction ``pitch`` from vertical (pi/2 = horizontal forward).  Elbow-up solution.
    Returns None when the point is unreachable or violates a joint limit."""
    tx = target_xyz[0] - BASE_TO_SHOULDER[0]
    ty = target_xyz[1] - BASE_TO_SHOULDER[1]
    tz = target_xyz[2] - BASE_TO_SHOULDER[2]
    r = math.hypot(tx, ty)
    q1 = math.atan2(ty, tx) if r > 1e-6 else 0.0
    # back off along the hand direction to get the wrist point
    wr = r - L3 * math.sin(pitch)
    wz = tz - L3 * math.cos(pitch)
    d2 = wr * wr + wz * wz
    d = math.sqrt(d2)
    if d > (L1 + L2) - 1e-4 or d < abs(L1 - L2) + 1e-4:
        return None
    cos_beta = (L1 * L1 + L2 * L2 - d2) / (2.0 * L1 * L2)      # interior elbow angle
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    q3 = math.pi - beta                                          # forearm folds forward
    alpha = math.atan2(wr, wz)                                   # shoulder->wrist, from vertical
    delta = math.atan2(L2 * math.sin(q3), L1 + L2 * math.cos(q3))
    q2 = alpha - delta
    q4 = pitch - q2 - q3
    q = (q1, q2, q3, q4)
    for val, key in zip(q, ("q1", "q2", "q3", "q4")):
        lo, hi = Q_LIMITS[key]
        if val < lo - 1e-6 or val > hi + 1e-6:
            return None
    return q


def interpolate(q_a, q_b, n: int):
    """n+1 joint-space waypoints from q_a to q_b (inclusive)."""
    return [tuple(a + (b - a) * i / n for a, b in zip(q_a, q_b)) for i in range(n + 1)]
