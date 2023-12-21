# novatel sensor fusion

## Clion

In the workspace, execute the command
```sh
colcon build --packages-select novatel_sensor_fusion --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```
Call `Tools | Compilation Database | Change Project Root` from the main menu and select the workspace directory (`dev_ws` in our case).

## Git
```shell
git filter-branch --force --index-filter "git rm -r --cached --ignore-unmatch data/gnss_imu_logs/" --prune-empty --tag-name-filter cat -- --all
```
