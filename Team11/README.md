# EE3305 TurtleBot3 Navigation Comparison

This project contains two ROS 2 workspaces for running and comparing TurtleBot3 Burger navigation in Gazebo, plus an additional controller attempt from another group member. The Original workspace uses Dijkstra path planning and Fixed Pure Pursuit control. The Improved workspace keeps the same basic approach but adds path refinement and two changes to the controller.

Both versions can be run separately or side by side. The recording scripts collect the robot's actual Gazebo data and save plots for comparison.

## Project folders

```text
EE3305_Two_Projects/
├── Original/                  Original planner and controller
│   ├── src/                   ROS 2 packages, maps and launch files
│   └── docs/                  Original course documentation
├── Improved/                  Modified planner and controller
│   └── src/                   ROS 2 packages, maps and launch files
├── controller_experiments/   Other group controller attempts (A04)
├── tools/                     Goal sender, data recorder and plotting scripts
├── results/                   Archived Gazebo runs and new test outputs
├── build_all.sh               Build both workspaces
├── run_original.sh            Start Original and its recorder
├── run_improved.sh            Start Improved and its recorder
├── send_goal.sh               Send the same goal to either or both runs
└── compare_again.sh           Regenerate plots from saved records
```

The `build/`, `install/` and `log/` folders are not included. ROS 2 creates them when the workspaces are built. Historical Gazebo test results are kept under `results/`; generated build folders and redundant development backups are omitted.

## Requirements

This project was set up for Ubuntu with **ROS 2 Jazzy**, Gazebo, RViz2 and the TurtleBot3 Gazebo simulation packages. You also need `colcon` to build the packages and Python 3 with `numpy` and `matplotlib` for plotting. The ROS packages used by the launch files include `turtlebot3_gazebo`, `ros_gz_sim` and `tf2_ros`.

The shell scripts use `/opt/ros/jazzy/setup.bash`. They must be run in an environment where ROS 2 Jazzy and the simulation dependencies are already installed. This ZIP includes the EE3305 source packages, not ROS 2 or Gazebo itself.

## Build and run

Extract the ZIP to your Ubuntu home directory, then open a terminal:

```bash
cd ~/EE3305_Two_Projects
./build_all.sh
```

Launch the Original version in one terminal:

```bash
cd ~/EE3305_Two_Projects
./run_original.sh
```

Launch the Improved version in a second terminal:

```bash
cd ~/EE3305_Two_Projects
./run_improved.sh
```

The two scripts use separate ROS domains and Gazebo partitions so they do not send commands to each other's robot:

| Workspace | ROS_DOMAIN_ID | GZ_PARTITION |
| --- | --- | --- |
| Original | 41 | `ee3305_original` |
| Improved | 42 | `ee3305_improved` |

Once both simulations have started and odometry is available, send the same goal from a third terminal:

```bash
cd ~/EE3305_Two_Projects
./send_goal.sh both 2.17 -0.11
```

Use `original` or `improved` instead of `both` to test only one version. You can also choose a goal using RViz. Running the two simulations at the same time needs more CPU and memory; running them one after the other is fine as well.

The scripts launch the **Python** planner, controller and behaviour nodes (`cpp:=False`). The C++ files remain in the course source tree, but they are not the implementations used for this comparison.

## Original and Improved controllers

Both controllers use a fixed lookahead distance and the same nominal forward speed. Pure Pursuit selects a point ahead on the planned path and calculates the turning command needed to approach it.

| Feature | Original | Improved |
| --- | --- | --- |
| Main controller | Fixed Pure Pursuit | Fixed Pure Pursuit |
| Lookahead distance | 0.10 m | 0.10 m |
| Nominal forward speed | 0.22 m/s | 0.22 m/s |
| Large heading error | Continues normal Pure Pursuit tracking | Stops forward movement and turns towards the lookahead point first |
| Switching back to tracking | Not applicable | Returns to Pure Pursuit when the heading error becomes small |
| Angular command changes | No extra change-rate limit | Limits how quickly the requested yaw rate can change |

The Improved controller adds **Heading Alignment** for situations where the robot is facing well away from the direction it needs to travel. It rotates in place when the heading error reaches 1.35 rad and resumes normal tracking once that error falls to 0.25 rad. Using different entry and exit thresholds avoids switching back and forth between the two modes.

It also adds **Angular Rate Limiting**. The controller limits the change in commanded angular velocity using an acceleration bound of 12 rad/s². This is meant to avoid sudden steering requests without reducing the normal 0.22 m/s forward command. The Improved controller does not use adaptive lookahead or a separate curvature-based speed controller.

The two workspaces also have different **planners**. Original uses weighted Dijkstra on the grid costmap. Improved starts with Dijkstra and then checks possible shortcuts, smooths remaining corners, and selects a safe candidate using distance, turning and obstacle-cost terms. It keeps the original path as a fallback if a proposed refinement fails its safety checks, and includes a small tolerance to reduce unnecessary path changes during replanning.

The planner and controller settings can be found in:

```text
Original/src/ee3305_bringup/params/run.yaml
Improved/src/ee3305_bringup/params/run.yaml
```

The Python implementations are in `src/ee3305_py/ee3305_py/planner.py` and `controller.py` under each workspace. The common speed limits are set to 0.22 m/s linear and 2.84 rad/s angular.

## Other controller attempt: A04

The `controller_experiments/A04/` folder contains a separate controller attempt contributed by another group member. We kept its original `controller.py` and `run.yaml` together so this work is included in the group submission. It is an **alternative, not the active controller** in either the Original or Improved workspace. The existing build and launch scripts still run the two versions described above.

A04 uses the same basic Pure Pursuit lookahead geometry, but changes the speed policy. It reduces the forward speed when the path curvature is high and limits how quickly forward speed can increase. The saved A04 settings use a 0.20 m lookahead and up to 0.40 m/s forward speed, unlike the 0.10 m and 0.22 m/s used for the Original/Improved comparison. A04 does **not** include the Improved controller's heading-alignment or angular-rate-limiting features.

See [`controller_experiments/A04/README.md`](controller_experiments/A04/README.md) for an explanation, the original settings, and notes on testing it separately. There were no A04-specific Gazebo logs in the submitted A04 ZIP, so the results archived under `results/` must not be presented as an A04 performance comparison.

## Recording and comparing results

The `tools/` folder contains the scripts used for the comparison:

| Tool | What it does |
| --- | --- |
| `send_goal.py` | Publishes a goal to the ROS domain selected by `send_goal.sh`. |
| `compare_record.py` | Subscribes to `/odom`, `/cmd_vel`, `/goal_pose`, `/path` and `/map`, then saves the run. It does not control the robot. |
| `compare_plot.py` | Reads the saved records and creates comparison figures and a summary. |
| `redraw.py` | Runs the plotting tool again on existing result folders. |

The archived results from previous trials are available under `results/`. See `results/README.md` for an index of the runs, including which folders contain both Original and Improved data. These historical recordings do not identify an exact controller code revision, so they should not all be presented as tests of the final controller.

The recorder starts automatically with `run_original.sh` or `run_improved.sh`. After a goal is sent, it records the robot's position, measured speed, commanded speed and angular velocity. It tries to place runs with similar starting positions and the same goal into one `results/route_.../` folder. A completed run is marked `reached`; an incomplete run is saved with its actual status rather than being counted as a success.

Each result folder can contain:

```text
results/route_YYYYMMDD_HHMMSS_xxxxxx/
├── Original_*.csv
├── Improved_*.csv
├── records.json
├── map_occupancy.json
├── summary.csv
├── trajectory_comparison.png
├── speed_comparison.png
├── clearance_comparison.png
└── command_vs_actual.png
```

The four PNG files show the travelled trajectories on the occupancy map, actual forward speed, approximate obstacle clearance, and commanded versus measured linear speed. If only one workspace has finished a run, the available plots may show only that run. They are updated when the matching run is added. Replotting an archived folder will overwrite its generated PNG and summary files; make a copy first if you want to keep the original plots unchanged.

To regenerate the figures after a test, run:

```bash
cd ~/EE3305_Two_Projects
./compare_again.sh
```

You can also pass a specific results folder to `compare_again.sh`.

The clearance plot uses distances estimated from the saved occupancy grid and an approximate robot radius. It is **not** a physical collision measurement. Also, the two workspaces change both planning and control, so any difference in the overall result cannot be attributed to the controller alone. For a controller-only test, both controllers would need the same path, start pose and goal.

## Notes

- Stop a running simulation with `Ctrl+C` in its terminal. The recorder attempts to save the run when stopped.
- If plots are missing but CSV files exist, check that Python `numpy` and `matplotlib` are installed, then run `./compare_again.sh`.
- Changes to the Python planner or controller should be made in the corresponding workspace's `src/` directory. Build outputs are generated files and should not be edited.
- The Original course documentation and attribution are kept under `Original/docs/`. The complete list of course packages is in the `package.xml` files.
