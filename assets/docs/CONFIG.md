# 🔨 Configuration Guide

In this file, we will introduce the configuration files in this codebase.

## 🦾 ***AirExo*-2**

In this codebase, we provide interface to retrieve encoder readings from ***AirExo*-2** in [`airexo/device/airexo.py`](../../airexo/device/airexo.py).

To configure an arm of ***AirExo*-2** with its corresponding [***AirExo*-2** joint configurations](#-airexo-2-joint-configurations), you can use the following format:

```yaml
defaults:
  - _self_
  - ../joint/left/airexo.yaml@airexo_joint_cfgs      # link to the airexo joint configurations.

_target_: airexo.device.airexo.AirExo
port: /dev/ttyUSB2                                   # port.
joint_cfgs: ${airexo_left_joint_cfgs}                # AirExo-2 joint configurations (required, need to obtain encoder indices from the configurations).
baudrate: 115200                                     # serial baudrate.
sleep_gap: 0.0016                                    # data communication sleep gap (or frequency), 0.0016 is the minimum gap in our settings (8 encoders).
logger_name: AirExo-left                             # logger name.
```

## 📷 Camera

In this codebase, we provide support for Intel RealSense RGB-D cameras. For other types of cameras, you could implement your own camera interface in [`airexo/device/camera.py`](../../airexo/device/camera.py), following the [`RealSenseRGBDCamera`](../../airexo/device/camera.py#L15) class.

To configure an Intel RealSense RGB-D camera, please add a configuration file named with the camera serial in [`airexo/configs/cameras/`](../../airexo/configs/cameras/), *e.g.*, [`105422061350.yaml`](../../airexo/configs/cameras/105422061350.yaml). Its content should be in the following format:

```yaml
defaults:
  - _self_
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled
    
_target_: airexo.device.camera.RealSenseRGBDCamera   # or your own camera interface.
serial: "105422061350"                               # serial number / or your own camera interface parameters.
frame_rate: 30                                       # frame rate / or your own camera interface parameters.
resolution: [1280, 720]                              # resolution / or your own camera interface parameters.
align: True                                          # RGB-D alignment / or your own camera interface parameters.
logger_name: Camera-main                             # logger name / or your own camera interface parameters.
```

## 🤖 Robot and Gripper

In this codebase, we provide support for Flexiv Rizon 4 robotic arms. Please download the [Flexiv RDK v1.5.1](https://github.com/flexivrobotics/flexiv_rdk/releases/tag/v1.5.1) and put the compiled python bindings in the [`airexo/device`](../../airexo/device/) folder. For other types of robots, you could implement your own robot interface in [`airexo/device/robot.py`](../../airexo/device/robot.py), following the [`Robot`](../../airexo/device/robot.py#L9) class. Please remember to implement the `get_state` function, which returns the required information of the robot as a dict, for data collection.

To configure a Flexiv Rizon 4 robotic arm with its corresponding [robot joint configurations](#-robot-joint-configurations), you can use the following format:

```yaml
defaults:
  - _self_
  - ../joint/left/robot.yaml@robot_joint_cfgs        # link to the robot joint configurations.

_target_: airexo.device.robot.Robot                  # or your own robot interface.
serial: Rizon4-062077                                # serial number / or your own robot interface parameters.
joint_cfgs: ${robot_joint_cfgs}                      # robot joint configurations (required).
gripper:                                             # gripper object (required).
    _target_: airexo.device.gripper.Robotiq2F85Gripper  
    port: /dev/ttyUSB0
    logger_name: Gripper-left
logger_name: Robot-left                              # logger name / or your own robot interface parameters.
min_joint_diff: 0.01                                 # the minimal joint difference / or your own robot interface parameters.
```

In this codebase, we provide support for Robotiq 2F-85 grippers. For other types of grippers, you could implement your own gripper interface in [`airexo/device/gripper.py`](../../airexo/device/gripper.py), following the [`Robotiq2F85Gripper`](../../airexo/device/gripper.py#L20) class. Please remember to implement the `get_state` function, which returns the required information of the gripper as a dict, for data collection.

To configure a Robotiq 2F-85 gripper, you can use the following format:

```yaml
_target_: airexo.device.gripper.Robotiq2F85Gripper   # or your own gripper interface.
port: /dev/ttyUSB0                                   # port / or your own gripper interface parameters.
logger_name: Gripper-left                            # logger name / or your own gripper interface parameters.
```

## 💪 Joint

In this codebase, we provide joint configurations for ***AirExo*-2** and the dual-arm robot. The left arm configurations are stored in the [`airexo/configs/joint/left/`](../../airexo/configs/joint/left/) folder, while the right arm configurations are stored in the [`airexo/configs/joint/right/`](../../airexo/configs/joint/right/) folder. 

Currently, we provide several sample configurations for our hardware platforms. These joint configurations are used for [in-the-wild demonstration collection via ***AirExo*-2**](#️-in-the-wild) and [the operation space adaptor](ADAPTOR.md#operation-space-adaptor) to transform the in-the-wild demonstrations into pseudo-robot demonstrations.
- [`airexo/configs/joint/left/robot.yaml`](../../airexo/configs/joint/left/robot.yaml) and [`airexo/configs/joint/right/robot.yaml`](../../airexo/configs/joint/right/robot.yaml): the joint configurations of the dual-arm robot platform.
- [`airexo/configs/joint/left/airexo.yaml`](../../airexo/configs/joint/left/airexo.yaml) and [`airexo/configs/joint/right/airexo.yaml`](../../airexo/configs/joint/right/airexo.yaml): the joint configurations of the ***AirExo*-2** system.
- [`airexo/configs/joint/left/calib.yaml`](../../airexo/configs/joint/left/calib.yaml) and [`airexo/configs/joint/right/calib.yaml`](../../airexo/configs/joint/right/calib.yaml): the joint calibration configurations between the ***AirExo*-2** system and the dual-arm robot platform. See [the joint calibration section](#-joint-calibration) for more details.

Also, we provide several sample configurations for the original AirExo platforms for the compatibility. These joint configurations are used for [teleoperation demonstration collection via the original AirExo](#-teleoperation).
- [`airexo/configs/joint/left/robot_old.yaml`](../../airexo/configs/joint/left/robot_old.yaml) and [`airexo/configs/joint/right/robot_old.yaml`](../../airexo/configs/joint/right/robot_old.yaml): the joint configurations of the dual-arm robot platform. See [the robot configuration section](#-robot-configurations) for more details.
- [`airexo/configs/joint/left/airexo_old.yaml`](../../airexo/configs/joint/left/airexo_old.yaml) and [`airexo/configs/joint/right/airexo_old.yaml`](../../airexo/configs/joint/right/airexo_old.yaml): the joint configurations of the original AirExo system. See [the ***AirExo*-2** configuration section](#-airexo-2-configurations) for more details.
- [`airexo/configs/joint/left/calib_old.yaml`](../../airexo/configs/joint/left/calib_old.yaml) and [`airexo/configs/joint/right/calib_old.yaml`](../../airexo/configs/joint/right/calib_old.yaml): the joint calibration configurations between the original AirExo system and the dual-arm robot platform. See [the joint calibration section](#-joint-calibration) for more details.

### 🤖 Robot Joint Configurations

To configure the joint information of one robot arm, please add a configuration file under the corresponding folder, *e.g.*, [`robot.yaml`](../../airexo/configs/joint/left/robot.yaml). Its content should be in the following format:

```yaml
num_joints: 8         # number of all joints (include gripper).
num_robot_joints: 7   # number of all robot joints (exclude gripper).
joint1:               # configurations for joint1.
  fixed: False        # whether to use a fixed value for this joint.
  fixed_value: 0      # if use a fixed value, specify the value.
  init_value: 73.14   # the initialization value of this joint before teleoperation.
  min: 200.5          # the minimum value of this joint, in degree.
  max: 159.5          # the maximum value of this joint, in degree.
  direction: 1        # the direction of this joint, 1 or -1.
  rad: True           # whether to transform degrees to radians.
  zero_centered: True # whether to transform the degrees in [min, max] to zero-centered equivalent degree.

# from joint2 to joint7: similar format.

joint8:               # gripper.
  fixed: False        # whether to use a fixed value.
  fixed_value: 0      # if use a fixed value, specify the value.
  init_value: 0       # the initialization value of the gripper before teleoperation.
  min: 0              # the minimum value of the gripper.
  max: 0.10           # the maximum value of the gripper.
```

Notably, this configuration format is also compatible with the original AirExo, *i.e.*, please use the same template to configure the robot platform for teleoperation demonstration collection, *e.g.*, [`robot_old.yaml`](../../airexo/configs/joint/left/robot_old.yaml).

### 🦾 ***AirExo*-2** Joint Configurations

To configure the joint information of one ***AirExo*-2** arm, please add a configuration file under the corresponding folder, *e.g.*, [`airexo.yaml`](../../airexo/configs/joint/left/airexo.yaml). Its content should be in the following format:

```yaml
num_joints: 8         # number of all joints (include gripper) .
num_robot_joints: 7   # number of all robot joints (exclude gripper).
joint1:               # configurations for joint1.
  id: 1               # encoder id.
  min: 335            # the minimum encoder value of this joint, in degree.
  max: 306.12305      # the maximum encoder value of this joint, in degree.
  direction: 1        # the direction of this joint, 1 or -1.
  rad: False          # whether the encoder value is represented in radians.

# from joint2 to joint8: similar format.

```

Notably, this configuration format is also compatible with the original AirExo, *i.e.*, please use the same template to configure the original AirExo for teleoperation demonstration collection, *e.g.*, [`airexo_old.yaml`](../../airexo/configs/joint/left/airexo_old.yaml).

### 🔬 Joint Calibration

To configure the joint calibration information between one ***AirExo*-2** arm and the corresponding robot arm, please add a configuration file under the corresponding folder, *e.g.*, [`calib.yaml`](../../airexo/configs/joint/left/calib.yaml). We support three types of joint calibration: `fixed`, `mapping` and `scaling`. 

- `fixed`: use the fixed value for this joint when transforming joint information from ***AirExo*-2** (or AirExo) to the robot, disregarding the joint encoder readings. For this type, you do not need to specify other parameters.
- `mapping`: use a fixed-scale mapping to map the joint of ***AirExo*-2** (or AirExo) to the robot joint via anchor values, *i.e.*, the anchor value of the ***AirExo*-2** joint corresponds to the anchor value of the robot joint. For this type, you need to specify the anchor values of both devices (`airexo` and `robot`). Typically, we set the robot anchor value to 0 and find the anchor value of the ***AirExo*-2** (*i.e.*, zero position) using calibration methods. For details, please refer to [the calibration guide](CALIB.md#initial-calibration).
- `scaling`: use a linear mapping to map the `[min, max]` joint range of ***AirExo*-2** (or AirExo) to the `[min, max]` joint range of the robot. For this type, you do not need to specify other parameters.

Its content should be in the following format:

```yaml
joint1:               # joint calibration configurations for joint1.
  type: mapping       # joint calibration type for joint1, one of [fixed, mapping and scaling].
  airexo: 143.3145599 # the anchor value of AirExo-2 joint1 for the joint mapping.
  robot: 0            # the anchor value of the robot joint1 for the joint mapping.

# from joint2 to joint7: similar format.

joint8:               # joint calibration configurations for joint8 (gripper).
  type: scaling       # joint calibration type for joint8 (gripper).
```

Notably, this configuration format is also compatible with the original AirExo, *i.e.*, please use the same template to configure the joint calibration between AirExo and the robot platform for teleoperation demonstration collection, *e.g.*, [`calib_old.yaml`](../../airexo/configs/joint/left/calib_old.yaml).

## 📷 Camera Calibration

### 🤖 Robot Camera Calibration

In this section, we use the following approach to calibrate the global camera *w.r.t.* robot base. By assuming fixed in-hand camera in each arms, we can obtain the transformations from each in-hand camera to its corresponding flange through hand-eye calibration, which is saved in [`airexo/helpers/constants.py` (L5-20)](../../airexo/helpers/constants.py#L5). The transformations from flange to the end-effector (gripper) is pre-defined by the gripper type in [`airexo/helpers/constants.py` (L23-30)](../../airexo/helpers/constants.py#L23). Then, for the global camera, we can use the ArUco calibration board to obtain the transformation from the markers to the global camera and the in-hand cameras. By recording the end-effector poses at calibration time, we can calculate the transformations from the global camera to the robot bases.

We provide a calibration script for this approach. By using it you need to [configure the robots and grippers](#-robot-and-gripper) and modify [`airexo/configs/calibration/robot.yaml`](../../airexo/configs/calibration/robot.yaml) as follows.

``` yaml
defaults:
  - _self_
  - ../joint/left/robot_old.yaml@robot_left_joint_cfgs  # left joint configuration for robot
  - ../joint/right/robot_old.yaml@robot_right_joint_cfgs  # right joint configuration for robot
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled

type: calibrator  # choices: calibrator, annotator, solver

calibrator:
  _target_: airexo.calibration.calibrator.Calibrator
  calib_type: robot  # choices: airexo, robot
  calib_path: /home/ubuntu/data/calib_robot/  # path to save calibration results
  camera_serials_global:  # list of global camera serials
  - "105422061350"
  camera_serial_inhand_left: "104122064161"  # serial of left inhand camera, required for robot calibration
  camera_serial_inhand_right: "104122061330"  # serial of right inhand camera, required for robot calibration
  device_left:  # left device (robot) configuration
    _target_: airexo.device.robot.Robot
    serial: Rizon4-062077
    joint_cfgs: ${robot_left_joint_cfgs}
    gripper:
      _target_: airexo.device.gripper.Robotiq2F85Gripper
      port: /dev/ttyUSB0
      logger_name: Gripper-left
    logger_name: Robot-left
    min_joint_diff: 0.01
  device_right:  # right device (robot) configuration
    _target_: airexo.device.robot.Robot
    serial: Rizon4R-062016
    joint_cfgs: ${robot_right_joint_cfgs}
    gripper:
      _target_: airexo.device.gripper.Robotiq2F85Gripper
      port: /dev/ttyUSB1
      logger_name: Gripper-right
    logger_name: Robot-right
    min_joint_diff: 0.01
  aruco_dict: DICT_7X7_250  # aruco dictionary
  aruco_idx: 0  # aruco index
  marker_length: 150  # marker length in mm
  vis: True  # whether to visualize the calibration results in GUI
  logger_name: Calibrator  # logger name
  config_camera_path: airexo/configs/cameras  # path to camera configuration files
```

### 🦾 ***AirExo*-2** Camera Calibration

In this section, we use a two-stage appraoch to calibrate the global camera *w.r.t.* ***AirExo*-2** base. Please refer to [the calibration guide](CALIB.md) for details

#### 1️⃣ Initial Calibration

Please set up the configuration files following [`airexo/configs/calibration/airexo.yaml`](../../airexo/configs/calibration/airexo.yaml). Please refer to [the initial calibration section in the calibration guide](CALIB.md#1️⃣-initial-calibration) for more details.

```yaml
defaults:
  - _self_
  - ../joint/left/airexo.yaml@airexo_left_joint_cfgs  # left joint configuration for AirExo-2
  - ../joint/right/airexo.yaml@airexo_right_joint_cfgs  # right joint configuration for AirExo-2
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled

type: calibrator  # choices: calibrator, annotator, solver

calibrator:
  _target_: airexo.calibration.calibrator.Calibrator
  calib_type: airexo  # choices: airexo, robot
  calib_path: /home/ubuntu/data/calib_airexo  # path to save calibration results
  camera_serials_global:  # list of global camera serials
  - "104122061602"
  camera_serial_inhand_left: null  # serial of left inhand camera
  camera_serial_inhand_right: null  # serial of right inhand camera
  device_left:  # left device (AirExo-2) configuration
    _target_: airexo.device.airexo.AirExo
    port: /dev/ttyUSB2
    joint_cfgs: ${airexo_left_joint_cfgs}
    baudrate: 115200
    sleep_gap: 0.0016
    logger_name: AirExo-left
  device_right:  # right device (AirExo-2) configuration
    _target_: airexo.device.airexo.AirExo
    port: /dev/ttyUSB3
    joint_cfgs: ${airexo_right_joint_cfgs}
    baudrate: 115200
    sleep_gap: 0.0016
    logger_name: AirExo-right
  aruco_dict: DICT_7X7_250  # aruco dictionary
  aruco_idx: 0  # aruco index
  marker_length: 150  # marker length in mm
  vis: True  # whether to visualize the calibration results in GUI
  logger_name: Calibrator  # logger name
  config_camera_path: airexo/configs/cameras  # path to camera configuration files
```

#### 2️⃣ Calibration via Differentiable Rendering

Please set up the configuration files following [`airexo/configs/calibration/airexo_solver_diff_ren.yaml`](../../airexo/configs/calibration/airexo_solver_diff_ren.yaml). Please refer to [the calibration via differentiable rendering section in the calibration guide](CALIB.md#2️⃣-calibration-via-differentiable-rendering) for more details.

```yaml
defaults:
  - _self_
  - ../joint/left/airexo.yaml@airexo_left_joint_cfgs  # left joint configuration for AirExo-2
  - ../joint/right/airexo.yaml@airexo_right_joint_cfgs  # right joint configuration for AirExo-2
  - ../joint/left/calib.yaml@left_calib_cfgs  # left joint calibration configuration
  - ../joint/right/calib.yaml@right_calib_cfgs  # right joint calibration configuration
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled

type: solver  # choices: calibrator, annotator, solver

solver:
  _target_: airexo.calibration.solver.AirExoCalibrationDifferntiableRenderingSolver

  calib_info:  # last calibration results (to be refined)
    _target_: airexo.calibration.calib_info.CalibrationInfo
    calib_path: /home/ubuntu/data/calib_airexo  # path to last calibration results
    calib_timestamp: 1737226610748  # calibration timestamp

  camera_serial: "104122061602"  # serial of global camera

  airexo_left_joint_cfgs: ${airexo_left_joint_cfgs}
  airexo_right_joint_cfgs: ${airexo_right_joint_cfgs}
  left_calib_cfgs: ${left_calib_cfgs}
  right_calib_cfgs: ${right_calib_cfgs}

  urdf_file: airexo/urdf_models/airexo/airexo_no_handle.urdf  # URDF file

  device: cuda  # choices: cpu, cuda
  max_translation: 0.03  # max translation in meters in terms of camera calibration
  max_rotation: 0.05243  # max rotation in radians in terms of camera calibration
  max_degree: 3  # max degree of rotation in terms of joint calibration
  width: 1280  # image width
  height: 720  # image height
  max_disparency: 0.1  # max disparency

solver_params:
  data_path: /home/ubuntu/data/airexo_calib_pair_data/  # path to the training data for calibration refinement
  save_path: /home/ubuntu/data/airexo_calib_solver/  # path to save the calibration refinement results
  num_iter: 1000  # number of iterations
  lr: 0.0001  # learning rate
  beta: 5  # balancing coefficient of mask loss and depth loss
```

#### 👀 (Optional) Calibration Visualizer & Annotator

**Real-Time**. 

```yaml
defaults:
  - _self_
  - ../joint/left/airexo.yaml@airexo_left_joint_cfgs  # left joint configuration for AirExo-2
  - ../joint/left/calib.yaml@left_calib_cfgs  # left joint calibration configuration
  - ../joint/right/airexo.yaml@airexo_right_joint_cfgs  # right joint configuration for AirExo-2
  - ../joint/right/calib.yaml@right_calib_cfgs  # right joint calibration configuration
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled

type: annotator  # choices: calibrator, annotator, solver

annotator:
  _target_: airexo.calibration.annotator.AnnotateAirExo2DCalibratorFromCamera

  calib_info:  # last calibration results (to be refined)
    _target_: airexo.calibration.calib_info.CalibrationInfo
    calib_path: /home/ubuntu/data/calib_airexo  # path to last calibration results
    calib_timestamp: 1737226610748  # calib timestamp

  config_camera_path: airexo/configs/cameras  # path to camera configuration files
  camera_serial: "104122061602"  # serial of global camera

  left_airexo:  # left device (AirExo-2) configuration
    _target_: airexo.device.airexo.AirExo
    port: /dev/ttyUSB2
    joint_cfgs: ${airexo_left_joint_cfgs}
    baudrate: 115200
    sleep_gap: 0.0016
    logger_name: AirExo-left
    
  right_airexo:  # right device (AirExo-2) configuration
    _target_: airexo.device.airexo.AirExo
    port: /dev/ttyUSB3
    joint_cfgs: ${airexo_right_joint_cfgs}
    baudrate: 115200
    sleep_gap: 0.0016
    logger_name: AirExo-right

  left_calib_cfgs: ${left_calib_cfgs}
  right_calib_cfgs: ${right_calib_cfgs}

  urdf_file: airexo/urdf_models/airexo/airexo.urdf  # URDF file
  
  near_plane: 0.01  # visualization parameters: near plane
  far_plane: 100.0  # visualization parameters: far plane
  initial_line_speed: 0.003  # initial translation adjustment speed in meters
  initial_angle_speed: 0.0087266463  # initial rotation/angle adjustment speed in radians
  line_step: 0.0005  # the translation adjustment step
  angle_step: 0.0010908308  # the rotation/angle adjustment step

annotator_params:
  save_path: null  # path to save the annotated calibration results; set to null for visualization only
```

**From Collected In-the-Wild Data**.

```yaml
defaults:
  - _self_
  - ../joint/left/airexo.yaml@airexo_left_joint_cfgs  # left joint configuration for AirExo-2
  - ../joint/left/calib.yaml@left_calib_cfgs  # left joint calibration configuration
  - ../joint/right/airexo.yaml@airexo_right_joint_cfgs  # right joint configuration for AirExo-2
  - ../joint/right/calib.yaml@right_calib_cfgs  # right joint calibration configuration
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled

type: annotator  # choices: calibrator, annotator, solver

annotator:
  _target_: airexo.calibration.annotator.AnnotateAirExo2DCalibratorFromData

  calib_info:  # last calibration results (to be refined)
    _target_: airexo.calibration.calib_info.CalibrationInfo
    calib_path: /home/ubuntu/data/calib_airexo  # path to last calibration results
    calib_timestamp: 1737226610748  # calib timestamp

  camera_serial: "104122061602"  # serial of global camera

  airexo_left_joint_cfgs: ${airexo_left_joint_cfgs}
  airexo_right_joint_cfgs: ${airexo_right_joint_cfgs}
  left_calib_cfgs: ${left_calib_cfgs}
  right_calib_cfgs: ${right_calib_cfgs}

  urdf_file: airexo/urdf_models/airexo/airexo.urdf  # URDF file
  
  near_plane: 0.01  # visualization parameters: near plane
  far_plane: 100.0  # visualization parameters: far plane
  initial_line_speed: 0.001  # initial translation adjustment speed in meters
  initial_angle_speed: 0.0087266463  # initial rotation/angle adjustment speed in radians
  line_step: 0.00002  # the translation adjustment step
  angle_step: 0.0010908308  # the rotation/angle adjustment step
  timestamp_step: 1  # the frame interval for visualizing in-the-wild data

annotator_params:
  data_path: /home/ubuntu/data/task_0013_wild/scene_0005  # path to the in-the-wild demonstrations being visualized
  save_path: /home/ubuntu/data/test_calib/  # path to save the annotated calibration results; set to null for visualization only
```

## 👀 Visualization

For visualization, we provide several [configurations](../../airexo/configs/vis/) in this codebase. Please refer to [the visualization guide](VIS.md) for sample usages. Please modify the configurations according to your own device and visualization requirements.

## 🛢️ Data Collection

In this section, we provide two types of demonstration collection: teleoperation data collection and in-the-wild data collection. Both collection codebase and configurations supports two versions of AirExo, *i.e.*, ***AirExo*-2** and the original AirExo. We also provide configuration files for the original AirExo in the same folder with the `_old` suffix.

### 🧰 Teleoperation

Please set up the configuration files following [`airexo/configs/collection/teleop.yaml`](../../airexo/configs/collection/teleop.yaml). 

```yaml
defaults:
  - _self_
  - ../joint/left/airexo.yaml@airexo_left_joint_cfgs  # left joint configuration for AirExo-2
  - ../joint/right/airexo.yaml@airexo_right_joint_cfgs  # right joint configuration for AirExo-2
  - ../joint/left/robot.yaml@robot_left_joint_cfgs  # left joint configuration for robot
  - ../joint/right/robot.yaml@robot_right_joint_cfgs  # right joint configuration for robot
  - ../joint/left/calib.yaml@left_calib_cfgs  # left joint calibration configuration
  - ../joint/right/calib.yaml@right_calib_cfgs  # right joint calibration configuration
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled

type: teleop  # choices: teleop, wild
path: "/home/ubuntu/data/"  # path to save data
cameras:  # list of camera serials
  - "105422061350"
  - "104122064161"
  - "104122061330"
camera_freq: 20  # camera frequency

controller:  # controller configuration
  _target_: airexo.collection.controller.DualArmController

  left_arm:  # left arm configuration
    _target_: airexo.collection.controller.SingleArmTeleoperator
    robot:  # robot configuration
      _target_: airexo.device.robot.Robot
      serial: Rizon4-062077
      joint_cfgs: ${robot_left_joint_cfgs}
      gripper: 
        _target_: airexo.device.gripper.Robotiq2F85Gripper
        port: /dev/ttyUSB0
        logger_name: Gripper-left
      logger_name: Robot-left
      min_joint_diff: 0.01
    airexo:  # AirExo configuration
      _target_: airexo.device.airexo.AirExo
      port: /dev/ttyUSB2
      joint_cfgs: ${airexo_left_joint_cfgs}
      baudrate: 115200
      sleep_gap: 0.0016
      logger_name: AirExo-left
    calib_cfgs: ${left_calib_cfgs}
    logger_name: TeleOP-left  # logger name

    max_vel_safe: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]  # max velocity for initialization and calibration
    max_acc_safe: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # max acceleration for initialization and calibration
    max_vel_rt: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # max velocity for real-time control
    max_acc_rt: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]  # max acceleration for real-time control

    use_impedance: True  # whether to use impedance control
    impedance_joint_stiffness: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]  # joint stiffness for impedance control
    impedance_joint_damping_ratio: [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]  # joint damping ratio for impedance control

  right_arm:  # right arm configuration
    _target_: airexo.collection.controller.SingleArmTeleoperator
    robot:  # robot configuration
      _target_: airexo.device.robot.Robot
      serial: Rizon4R-062016
      joint_cfgs: ${robot_right_joint_cfgs}
      gripper: 
        _target_: airexo.device.gripper.Robotiq2F85Gripper
        port: /dev/ttyUSB1
        logger_name: Gripper-right
      logger_name: Robot-right
      min_joint_diff: 0.01
    airexo:  # AirExo configuration
      _target_: airexo.device.airexo.AirExo
      port: /dev/ttyUSB3
      joint_cfgs: ${airexo_right_joint_cfgs}
      baudrate: 115200
      sleep_gap: 0.0016
      logger_name: AirExo-right
    calib_cfgs: ${right_calib_cfgs}
    logger_name: TeleOP-right  # logger name

    max_vel_safe: [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]  # max velocity for initialization and calibration
    max_acc_safe: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]  # max acceleration for initialization and calibration
    max_vel_rt: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # max velocity for real-time control
    max_acc_rt: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]  # max acceleration for real-time control

    use_impedance: True  # whether to use impedance control
    impedance_joint_stiffness: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]  # joint stiffness for impedance control
    impedance_joint_damping_ratio: [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]  # joint damping ratio for impedance control

lowdim_collectors:  # list of low-dimensional collectors
  - _target_: airexo.collection.collector.LowdimCollector
    name: "robot_left"  # key of the collector
    controller_key: ["left_arm", "robot"]  # key link trajectory to the object, whose state will be collected. This object should implement `get_state()`
    freq: 100  # frequency
    buffer_size: 1000  # buffer size
  - _target_: airexo.collection.collector.LowdimCollector
    name: "robot_right"
    controller_key: ["right_arm", "robot"]
    freq: 100
    buffer_size: 1000
  - _target_: airexo.collection.collector.LowdimCollector
    name: "gripper_left"
    controller_key: ["left_arm", "robot", "gripper"]
    freq: 20
    buffer_size: 1000
  - _target_: airexo.collection.collector.LowdimCollector
    name: "gripper_right"
    controller_key: ["right_arm", "robot", "gripper"]
    freq: 20
    buffer_size: 1000
```

### 🏞️ In-the-Wild

Please set up the configuration files following [`airexo/configs/collection/wild.yaml`](../../airexo/configs/collection/wild.yaml).

```yaml
defaults:
  - _self_
  - ../joint/left/airexo.yaml@airexo_left_joint_cfgs  # left joint configuration for AirExo-2
  - ../joint/right/airexo.yaml@airexo_right_joint_cfgs  # right joint configuration for AirExo-2
  - ../joint/left/robot.yaml@robot_left_joint_cfgs  # left joint configuration for robot
  - ../joint/right/robot.yaml@robot_right_joint_cfgs  # right joint configuration for robot
  - ../joint/left/calib.yaml@left_calib_cfgs  # left joint calibration configuration
  - ../joint/right/calib.yaml@right_calib_cfgs  # right joint calibration configuration
  - override hydra/hydra_logging: disabled  
  - override hydra/job_logging: disabled

type: wild  # choices: teleop, wild
path: "/home/ubuntu/data/"  # path to save data
cameras:  # list of camera serials
  - "104122061602"
camera_freq: 20  # camera frequency

controller:  # controller configuration
  _target_: airexo.collection.controller.DualArmController  # we do not need a controller for in-the-wild data collection, thus we use a dummy controller

  left_arm:  # left arm configuration
    _target_: airexo.collection.controller.SingleArmDummyController
    airexo:  # AirExo configuration
      _target_: airexo.device.airexo.AirExo
      port: /dev/ttyUSB2
      joint_cfgs: ${airexo_left_joint_cfgs}
      baudrate: 115200
      sleep_gap: 0.0016
      logger_name: AirExo-left
    logger_name: DummyCtrl-left  # logger name

  right_arm:  # right arm configuration
    _target_: airexo.collection.controller.SingleArmDummyController
    airexo:  # AirExo configuration
      _target_: airexo.device.airexo.AirExo
      port: /dev/ttyUSB3
      joint_cfgs: ${airexo_right_joint_cfgs}
      baudrate: 115200
      sleep_gap: 0.0016
      logger_name: AirExo-right
    logger_name: DummyCtrl-right  # logger name

lowdim_collectors:  # list of low-dimensional collectors
  - _target_: airexo.collection.collector.LowdimCollector
    name: "airexo_left"  # key of the collector
    controller_key: ["left_arm", "airexo"]  # key link trajectory to the object, whose state will be collected. This object should implement `get_state()`
    freq: 60  # frequency of data collection
    buffer_size: 1000  # buffer size
  - _target_: airexo.collection.collector.LowdimCollector
    name: "airexo_right"
    controller_key: ["right_arm", "airexo"]
    freq: 60
    buffer_size: 1000
```

## 🔃 Adaptors
The AirExo-2 system utilizes specialized adaptors to transform in-the-wild observations into pseudo-robot observations, bridging the visual and operational gaps between human demonstrations and robot execution. This document details the configuration of these adaptors.

### 🖼️ Image Adaptor
The image adaptor processes visual information through a comprehensive pipeline to transform in-the-wild images into realistic robot demonstrations.

#### 🤖 Image Generation

This component leverages ControlNet to generate photorealistic robot images from rendered robot frames. After [training the model](../../utils/controlnet/) with platform-specific paired samples, please set up the configuration files following [airexo/configs/adaptor/controlnet_inference.yaml](../../airexo/configs/adaptor/controlnet_inference.yaml).

```yaml
paths:
  base_path: null  # Base path from environment variable
  scene_name: null  # Scene name from environment variable
  camera_id: "cam_104122061602"  # Default camera ID

dirs:
  color: "color"  # Color frames directory 
  depth: "depth"  # Depth frames directory
  render_robot: "render_robot"  # Rendered robot directory
  output: "color_controlnet"  # Output directory

model:
  config_path: "ControlNet/models/cldm_v15.yaml"  # path to controlnet configurations
  checkpoint_path: "checkpoint.ckpt"  # path to controlnet checkpoint
  device: "cuda"  # device

inference:
  prompt: "robotic arms, dual arm, industrial robotic manipulator, metallic silver color, mechanical joints, precise mechanical details, gripper end effector, high quality photo, photorealistic, clear and sharp details"  # controlnet prompt
  num_samples: 1
  image_size: 512  # image size
  steps: 50  # diffusion steps
  scale: 9.0  # guidance scale
  seed: null  # seed
  batch_size: 23  # batch size
```


#### 🎨 Color Inpainting

This component removes human embodiment information from the in-the-wild images using the ProPainter video inpainting model. Please set up the configuration files following [airexo/configs/adaptor/color_adaptor.yaml](../../airexo/configs/adaptor/color_adaptor.yaml).

```yaml
paths:
  base_path: null  # Base path from environment variable or default
  scene_name: null   # Scene name from environment variable or default
  video: ${.base_path}/${.scene_name}/cam_104122061602/color  # Input video path or image folder
  mask: ${.base_path}/${.scene_name}/cam_104122061602/sam_mask  # Path of mask(s) or mask folder
  render_airexo_path: ${.base_path}/${.scene_name}/cam_104122061602/render_airexo/mask  # Optional path to render mask
  output: ${.base_path}/propainter_combine  # Output folder
  output_pics: ${.base_path}/${.scene_name}/cam_104122061602/color_inpainting  # Path for output pictures
  model_dir: 'weights'  # Directory for model weights

video:
  resize_ratio: 1.0  # Resize scale for processing video
  height: -1  # Height of the processing video, -1 for original
  width: -1  # Width of the processing video, -1 for original
  save_fps: 24  # Frame per second
  save_frames: false  # Save output frames

outpainting:
  scale_h: 1.0  # Outpainting scale of height
  scale_w: 1.2  # Outpainting scale of width

mode: video_inpainting  # choices: video_inpainting, video_outpainting
processing:
  mask_dilation: 4  # Mask dilation for video and flow masking
  ref_stride: 10  # Stride of global reference frames
  neighbor_length: 10  # Length of local neighboring frames
  subvideo_length: 80  # Length of sub-video for long video inference
  raft_iter: 20  # Iterations for RAFT inference
runtime:
  fp16: false  # Use half precision during inference
```

After generating both inpainted images and ControlNet robot images, this component combines them using the robot masks to create the final pseudo-robot images with high visual fidelity. Please set up the configuration files following [airexo/configs/adaptor/controlnet_inpainting.yaml](../../airexo/configs/adaptor/controlnet_inpainting.yaml).

```yaml
paths:
  base_path: null  # Base path from environment variable or default
  scene_name: null   # Scene name from environment variable or default
  camera_id: "cam_104122061602"  # Default camera ID

dirs:
  inpainting: "color_inpainting"  # Directory containing inpainted images
  controlnet: "color_controlnet"  # Directory containing controlnet generated images
  mask: "render_robot/mask"  # Directory containing mask images
  output: "color_controlnet_test"  # Directory where combined results will be saved

verbose: true        
```

### 📏 Depth Adaptor

The Depth Adaptor processes depth information from RGB-D cameras, replacing human depth data with appropriate background and robot depth values. It requires a reference depth image of the empty workspace as a universal background. Please set up the configuration files following [airexo/configs/adaptor/depth_inpainting.yaml](../../airexo/configs/adaptor/depth_adaptor.yaml).

```yaml
paths:
  base_path: null  # Base path from environment variable
  scene_name: null  # Scene name from environment variable
  camera_id: "cam_104122061602"  # Default camera ID
  empty_depth_dir: "/data/empty/cam_104122061602/depth"  # Path to empty depth directory

dirs:
  color: "color"  # Color frames directory
  depth: "depth"  # Depth frames directory
  sam_mask: "sam_mask"  # SAM mask directory
  sam_base_mask: "sam_base_mask"  # SAM base mask directory
  render_robot: "render_robot"  # Rendered robot directory
  render_airexo: "render_airexo"  # Rendered AirExo directory
  output: "depth_inpainting"  # Output directory

processing:
  use_airexo_mask: true  # Whether to use airexo mask processing

visualization:
  viz_empty: true  # Whether to visualize empty background depth
  viz_path: "./empty_depth_viz.png"  # Path to save empty depth visualization
```
