# Challenge C100 — Quaternion/Rotation Echo

## Objective
Understand basic 3D orientation representation in ROS2 and practice converting and publishing quaternions.

## Task
- Subscribe to the topic `c100/orientation`
- Decode the quaternion (roll, pitch, yaw or axis-angle)
- Compute the inverse rotation
- Publish the inverse quaternion on `c100/inverse`

The node will emit the flag when the inverse is correct.

## Concepts
- `geometry_msgs/msg/Quaternion`
- Quaternion math (inverse)
- ROS2 publishers & subscribers
- Spinning and timers
- Coordinate frame reasoning
