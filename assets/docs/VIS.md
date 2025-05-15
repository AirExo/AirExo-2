# 👀 Visualization

In this codebase, we provide several visualization scripts for both ***AirExo*-2** and the robot platform. The real-time visualization process reads the camera frames and the ***AirExo*-2** / robot information and visualize the URDF model in the scene. Please follow [the configuration guide](CONFIG.md#-visualization) to set up the configuration files.

```bash
python -m airexo.tests.vis --config-name=[airexo/airexo_rgb/airexo_robot/all/robot/robot_sep]
```

- [`airexo`](../../airexo/configs/vis/airexo.yaml): 3D point cloud visualization of the ***AirExo*-2** on the ***AirExo*-2** platform.
- [`airexo_rgb`](../../airexo/configs/vis/airexo_rgb.yaml): 2D visualization of the ***AirExo*-2** on the ***AirExo*-2** platform.
- [`airexo_robot`](../../airexo/configs/vis/airexo_robot.yaml): 3D visualization of the robot replacing ***AirExo*-2** on the ***AirExo*-2** platform.
- [`all`](../../airexo/configs/vis/all.yaml): 3D visualization of both ***AirExo*-2** and the corresponding robot on the ***AirExo*-2** platform.
- [`robot`](../../airexo/configs/vis/robot.yaml): 3D visualization of the robot on the robot platform.
- [`robot_sep`](../../airexo/configs/vis/robot_sep.yaml): 3D visualization of the robot on the robot platform, separate visualization for two arms.
