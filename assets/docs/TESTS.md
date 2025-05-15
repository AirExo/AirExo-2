# 🕵️ Testing Scripts

We also provide several testing scripts in this codebase to facilitate modular testing.

## 🦾 ***AirExo*-2**

### Test Encoder

During the assembly process, we recommend checking the status of the encoders each time a new encoder is installed on a joint. Please use the following testing script to check the status of the encoders. You might need to modify the [configurations](../../airexo/configs/tests/airexo.yaml) to accomodate your own circumstances.

```bash
python -m airexo.tests.airexo
```

### Test End-Effector Poses

Please use the following testing script to check the end-effector poses calculated via forward kinematics after the assembly process. You might need to modify the [configurations](../../airexo/configs/tests/airexo_tcp.yaml) to accomodate your own circumstances.

```bash
python -m airexo.tests.airexo_tcp
```

## 🤖 URDF

We provide several URDF files for different use, as well as their visualization scripts and [configurations](../../airexo/configs/tests/urdf/).

### Robot (Single-Arm)

```bash
python -m airexo.tests.urdf_robot_single --config-name=[left_robot/left_robot_inhand/right_robot/right_robot_inhand]
```

- `airexo/urdf_models/robot/left_robot.urdf` [[config]](../../airexo/configs/tests/urdf/left_robot.yaml): the URDF of the left robot arm.
- `airexo/urdf_models/robot/left_robot_inhand.urdf` [[config]](../../airexo/configs/tests/urdf/left_robot_inhand.yaml): the URDF of the left robot arm (with inhand camera).
- `airexo/urdf_models/robot/right_robot.urdf` [[config]](../../airexo/configs/tests/urdf/right_robot.yaml): the URDF of the right robot arm.
- `airexo/urdf_models/robot/right_robot_inhand.urdf` [[config]](../../airexo/configs/tests/urdf/right_robot_inhand.yaml): the URDF of the right robot arm (with inhand camera).

### Robot (Dual-Arm)

```bash
python -m airexo.tests.urdf_robot --config-name=[robot/robot_inhand]
```

- `airexo/urdf_models/robot/robot.urdf` [[config]](../../airexo/configs/tests/urdf/robot.yaml): the URDF of the dual-arm robot.
- `airexo/urdf_models/robot/robot_inhand.urdf` [[config]](../../airexo/configs/tests/urdf/robot_inhand.yaml): the URDF of the dual-arm robot (with inhand cameras).

### ***AirExo*-2**

```bash
python -m airexo.tests.urdf_airexo --config-name=[airexo/airexo_no_handle]
```

- `airexo/urdf_models/airexo/airexo.urdf` [[config]](../../airexo/configs/tests/urdf/airexo.yaml): the URDF of ***AirExo*-2**.
- `airexo/urdf_models/airexo/airexo_no_handle.urdf` [[config]](../../airexo/configs/tests/urdf/airexo_no_handle.yaml): the URDF of ***AirExo*-2** (without handle).

## 🎮 Teleoperation

In the codebase, data collection is integrated with the teleoperation (control) process to improve collection performance. We also provide testing scripts for the teleoperation process to facilitate debugging, as well as their [configurations](../../airexo/configs/tests/teleop/).

```bash
python -m airexo.tests.teleop --config-name=[both/left/right/both_old/left_old/right_old]
```

- [`airexo/configs/tests/teleop/both.yaml`](../../airexo/configs/tests/teleop/both.yaml): test the teleoperation of both arms.
- [`airexo/configs/tests/teleop/left.yaml`](../../airexo/configs/tests/teleop/left.yaml): test the teleoperation of the left arm.
- [`airexo/configs/tests/teleop/right.yaml`](../../airexo/configs/tests/teleop/right.yaml): test the teleoperation of the right arm.

The configurations with suffices `_old` are also provided for testing. They are compatible with the original AirExo.

## 🎞️ Renderer

We provide testing scripts to verify the rendering process. By using the calibration results as inputs, the scripts can render AirExo-2 or the robot in the camera frames. The [configurations](../../airexo/configs/tests/renderer/) are also provided.

The following command [[config]](../../airexo/configs/tests/renderer/airexo.yaml) tests the rendering process of ***AirExo*-2**.

```bash
python -m airexo.tests.renderer_airexo
```

The following command [[config]](../../airexo/configs/tests/renderer/robot.yaml) tests the rendering process of the robot (as a dual-arm system).

```bash
python -m airexo.tests.renderer_robot
```

The following command [[config]](../../airexo/configs/tests/renderer/robot.yaml) tests the rendering process of the robot (as two arms separately). This configuration is actually used in the codebase to address installation errors and improve the calibration accuracy of the robot platform.

```bash
python -m airexo.tests.renderer_robot_sep
```
