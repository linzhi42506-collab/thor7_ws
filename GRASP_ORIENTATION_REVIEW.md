# THOR7 downward pregrasp review — 2026-09-10

## Geometry and scope

The actual read-only and SERVOJ robot descriptions include the expanded
`src/thor7v3_description/urdf/thor7v3_robot.urdf`. Its fixed chain is:

* link6 -> ft_sensor_mount_link: identity.
* ft_sensor_mount_link -> ag145_gripper_base_link: Z = 0.0387 m,
  yaw = -pi/2.
* ag145_gripper_base_link -> tool0: Z = 0.2000 m, no rotation.

Thus link6 -> tool0 is 238.7 mm along local +Z, with a -90 degree yaw.
The two knuckles are positioned on opposite local X sides and rotate about
local Y, so tool0 X is the opening direction and +Z extends through the fingers.

The supplied `/home/landau/Downloads/WF.STEP` was opened through OpenCascade
(54 solids). It includes Thor7_J6 and the AG145 assembly/fingers. This is
assembly geometry, not a measured grasp TCP definition. The existing 200 mm
gripper-base-to-tool0 modeling offset is retained. A distal finger boundary
must not automatically be substituted for the desired grasp-center TCP.
URDF FK validation measures model/feedback consistency, not physical TCP accuracy.

## New behavior

The goal orientation is Rz(grasp_yaw) Rx(pi) in base_link. At yaw zero,
tool0 +Z points vertically down, and the opening axis is parallel to base X.
The target detector provides a point, not an object orientation; this yaw is
an explicit operator choice, not inferred from YOLO.

Preparing pregrasp constructs one reviewed trajectory with two phases:

1. Collision-aware OMPL alignment at the current XY. The alignment height is
   max(current tool0 Z, pregrasp Z + alignment_clearance). Interpolated FK checks
   reject a drop below the initial tool0 height (3 mm numerical allowance).
   A base-frame tool0 PositionConstraint supplies that same floor to OMPL
   during search, using a 10 m box extending upward from the floor (well beyond
   this robot's reach). The post-check remains in place. Path rejection reports
   the stage, waypoint, measured Z, required minimum Z and orientation error.
   The tilted initial state is allowed here; no impossible downward constraint
   is imposed on the initial rotation transition.
2. Collision-aware Cartesian approach to pregrasp with an explicit downward
   orientation constraint. This is a straight approach from the alignment
   position, not necessarily a vertical approach. Vertical descent is a
   separate prepare_near/execute_near operation.

Incomplete Cartesian paths are rejected. Approach/descent joint jumps and
interpolated orientation/height are checked. Every endpoint must pass FK
position (default 3 mm) and orientation (default 3 degrees) checks before
publication; execution success also requires feedback FK to pass. The
clearance success messages now reflect configured values instead of hardcoded
50/5 mm. Existing workspace, fresh-target, start-state, age and single-use
execution checks remain in force.

The new parameters are grasp_yaw (radians), orientation_tolerance (radians),
endpoint_position_tolerance (meters), and alignment_clearance (meters).
There is no fallback to a tilted grasp if alignment or approach fails.

RViz defaults enable the independent query-goal controls (0.1 m marker size),
render the planned robot and scene robot opaque. Existing running RViz instances
must reload the config to see these settings. Empty DisplayTrajectory is
published on invalidation; actual clearing depends on the RViz display.

## First review

Stop only the old guarded planning node and start the rebuilt version with
execution disabled. Keep a single current camera TF and vision publisher.

```bash
source /opt/ros/humble/setup.bash
source /home/landau/Documents/thor7_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch thor7_moveit_demo vision_grasp_pregrasp_guarded.launch.py \
  enable_execution:=false \
  pregrasp_clearance:=0.080 near_clearance:=0.030 \
  grasp_yaw:=0.0 velocity_scaling:=0.03 acceleration_scaling:=0.03 \
  descent_velocity_scaling:=0.02 descent_acceleration_scaling:=0.02
```

In another sourced terminal:

```bash
ros2 service call /thor7_vision_grasp/prepare_pregrasp std_srvs/srv/Trigger '{}'
```

Inspect both the alignment and approach segments. Planning refusal is a
reason to inspect staging position/yaw/collisions, not to loosen guards blindly.
No real motion or live service call was performed as part of this change.

Follow-up: refreshed FK caches before accessing const RobotState transforms.
CurrentStateMonitor may update joints without updating cached link poses;
Release-mode const getters otherwise silently return old transforms. Added
a regression test with dirty joints, an offset base frame and high/low poses.
