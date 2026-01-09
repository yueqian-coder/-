# MRPC-2025 Homework Report (Draft)

## Task 1: Coordinate Transform

Given the body-to-end-effector rotation

  R_BD = [[cos(w t), -sin(w t) cos(a),  sin(w t) sin(a)],
          [sin(w t),  cos(w t) cos(a), -cos(w t) sin(a)],
          [0,          sin(a),          cos(a)]]

with w = 0.5 rad/s and a = pi/12, the end-effector rotation in the world frame is

  R_WD = R_WB * R_BD

where R_WB is built from the quaternion in tracking.csv (qx, qy, qz, qw). The
rotation matrix is converted to a quaternion (qx, qy, qz, qw) and the sign is
flipped if qw < 0 to enforce qw >= 0. The outputs are:

- solutions/end_effector_quaternion.csv
- solutions/end_effector_quaternion.png (qx and qy vs time)

## Task 2: Open Questions

### 2.1 A* planning and resolution

The A* loop maintains an open list ordered by f = g + h, where h is the
Euclidean distance heuristic with a small tie-breaker. Grid resolution affects
both search speed and path quality: a smaller resolution increases nodes and
runtime but yields tighter paths; a larger resolution speeds up search but can
increase path length and reduce clearance. In this implementation, the path
is simplified after search to reduce redundant waypoints. As an optional
improvement, I added a small vertical-movement penalty in the edge cost to
discourage unnecessary altitude changes and produce smoother paths.

### 2.2 SO3 control attitude generation

The SO3 controller forms the desired total force using position, velocity and
acceleration errors (gain matrices kx, kv) plus gravity compensation. The
desired body z-axis is aligned with the normalized force vector. A desired yaw
vector b1d is formed from the target yaw, then b2 and b1 are recovered by cross
products to assemble the rotation matrix. The rotation is converted to a
quaternion and sent to the simulator. Higher kx/kv increase responsiveness but
can cause overshoot; lower gains improve smoothness but increase tracking error.

### 2.3 Dynamics modeling with disturbances

The translational dynamics are

  v_dot = (R * [0,0,thrust] + external_force - drag) / m - [0,0,g]

where drag is proportional to speed squared and points opposite velocity. The
rotational dynamics are

  omega_dot = J^{-1} * (moments + external_moment - omega x (J * omega))

which includes gyroscopic coupling and external moments.

## Task 3 (Optional): Differential Flatness

Trajectory:

  x = 10 cos t / (1 + sin^2 t)
  y = 10 sin t cos t / (1 + sin^2 t)
  z = 10,  t in [0, 2 pi]

Velocity and acceleration are computed analytically. The thrust direction is

  b3 = (a + g e3) / ||a + g e3||

The yaw is aligned with velocity, so b1d = [cos(yaw), sin(yaw), 0]. Then

  b2 = (b3 x b1d) / ||b3 x b1d||
  b1 = b2 x b3
  R = [b1 b2 b3]

The quaternion is normalized and flipped to keep qw >= 0, with continuity
preserved by sign checks. Output file:

- solutions/df_quaternion.csv

The ROS package quadrotor_df generates this CSV using C++ and Eigen.

## Evaluation Results (demo.launch)

Tuning used:
- Theta*-style parent rewire with line-of-sight checks in A* (Astar_searcher.cpp)
- shortcut path with line-of-sight checks (trajectory_generator_node.cpp)
- timeAllocation scale: 1.2x (trajectory_generator_node.cpp)
- position gains: kx = (20, 20, 16), kv = (12, 12, 10) (so3_control_nodelet.cpp)

Latest run (solutions/result.txt):
- RMSE: 0.021383851990198346
- Total time: 23.179882049560547
- Path length: 28.997778154811613
- Collision: 0
- Overall score: 14.7123024389141

Note: the map generator is randomized, so scores can vary between runs
(best observed with the same settings was ~14.71).
