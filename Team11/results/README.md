# Recorded Gazebo tests

These folders contain the previous TurtleBot3 Gazebo runs from the original project archive. They are kept so the test history can be checked alongside the submitted code. No recorded CSV, JSON or PNG was changed during cleanup.

Each folder contains the available run CSV files, `records.json`, `map_occupancy.json`, `summary.csv` and comparison figures. The CSV files hold recorded odometry and control commands. The images show the actual travelled route, measured speed, estimated clearance and commanded versus measured speed. A run marked `near_goal_stopped` should not be reported as `reached`.

| Run folder | Original status | Improved status |
| --- | --- | --- |
| `route_20261026_012422_608302` | near_goal_stopped | reached |
| `route_20261026_020905_828374` | near_goal_stopped | reached |
| `route_20261026_162008_979705` | - | near_goal_stopped |
| `route_20261026_200202_708994` | reached | - |
| `route_20261026_200419_604051` | - | reached |
| `route_20261026_200635_760639` | - | reached |
| `route_20261026_202021_378164` | near_goal_stopped | reached |
| `route_20261026_202310_568593` | near_goal_stopped | reached |

These runs were recorded at different stages and do not contain a verified source revision identifier. Do not assume every historical Improved run used the final submitted controller. Start positions, headings and goals may differ, so a paired folder is useful for visual comparison but is not always a controlled test of the controller alone.

`group_index.json` is the original recorder's history of grouping requests. It can mention older folders that are not present in this archive; the table above lists only the available result folders.

To view past results, open the PNG or CSV files directly. The plotting scripts can recreate figures from the CSV files, but they rewrite the existing PNGs and `summary.csv`. Make a copy of a result folder before regenerating plots if you want to keep the archived images unchanged.

The `*.lock` files have been left out because they are empty runtime locks, not measurements. ROS `build/`, `install/` and `log/` folders were not included because they are generated build artifacts rather than the recorded navigation tests in this folder.
