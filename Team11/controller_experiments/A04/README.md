# A04 Controller - Alternative Group Attempt

This folder keeps a controller experiment contributed by another member of our EE3305 group. We included the files in the same submission so the different approaches are easy to inspect. The files are preserved as received, including their existing comments.

## Included files

- `controller.py` - Python ROS 2 controller node from the A04 attempt.
- `run.yaml` - the parameter settings submitted with that controller.

These files do not form a complete ROS 2 workspace on their own. The main `Original/` and `Improved/` folders contain the maps, planner, launch files, and other packages needed for a complete navigation run.

## What A04 changes

A04 is another version of Pure Pursuit. It finds a lookahead point on the supplied path and calculates the turning curvature as usual. Instead of always requesting the same forward speed, it makes the forward speed smaller on sharper turns:

`target_speed = min(lookahead_lin_vel, 0.60 / max(abs(curvature), 1e-6))`

The `0.60` value is the angular-speed target used within A04's adaptive-speed calculation. The code also limits *increases* in the commanded forward speed to `0.50 m/s^2`. It allows immediate decreases when the target speed becomes lower. The angular command is then calculated from the actual commanded linear speed and the Pure Pursuit curvature, and is clipped to the configured maximum angular speed.

A04's supplied `run.yaml` sets a 0.20 m lookahead, up to 0.40 m/s forward speed, and a 2.0 rad/s angular-speed limit. These settings are different from the main project's 0.10 m lookahead and 0.22 m/s nominal forward speed.

## How this differs from our submitted Improved controller

| | Improved (main version) | A04 (alternative attempt) |
| --- | --- | --- |
| Basis | Fixed Pure Pursuit | Pure Pursuit |
| Forward speed | Fixed nominal 0.22 m/s | Curvature-dependent, up to 0.40 m/s |
| Large heading mismatch | Stop and align before tracking | No separate alignment mode |
| Command smoothing | Limit changes in angular velocity | Limit increases in linear velocity |
| Lookahead | 0.10 m | 0.20 m |

A04 may reduce the forward-speed demand on tight turns, but it does not directly address the large initial heading-error case studied with the main Improved controller. The source files alone do not show whether it performed better or worse in Gazebo.

## Testing A04 with the existing project

The main launch scripts **do not use A04 automatically**. To test it without changing the submitted Original or Improved versions:

1. Make a separate copy of the `Improved/` workspace, for example `Improved_A04_Test/`.
2. Replace only the **Python** `ee3305_py/ee3305_py/controller.py` in that copy with this folder's `controller.py`.
3. In the copied workspace's `src/ee3305_bringup/params/run.yaml`, replace only the `controller: ros__parameters` section with the settings from A04's `run.yaml`. **Do not overwrite the entire YAML file**, because doing so would remove the Improved planner's path-refinement settings.
4. Build the copied workspace with `colcon build --symlink-install` in an installed ROS 2 Jazzy environment. Launch using the regular `ee3305_bringup` launch file with `cpp:=False`, in a separate ROS domain and Gazebo partition if another test is already running.
5. Compare all versions using the same map, goal, start pose and heading. For a fair *controller-only* test, use identical paths and compatible speed/lookahead settings, or clearly report the differences.

No test data specific to A04 was supplied in `EE3305_Controller_A04.zip`. The historical data under the project's `results/` folder comes from other trials and must not be attributed to A04.

## Status

**Archived alternative; not integrated into the default launch; no A04-specific Gazebo results supplied.** The original source files have not been edited.
