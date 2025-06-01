# AirExo-2: Scaling up Generalizable Robotic Imitation Learning with Low-Cost Exoskeletons

[Paper](https://arxiv.org/abs/2503.03081) | [Project Page](http://airexo.tech/airexo2/) | [***AirExo*-2** Hardware](https://github.com/AirExo/AirExo-2/tree/main/assets/models/AirExo-2) | [Sample Data](https://drive.google.com/drive/folders/1xfJgn1qVLBgNhaFq7sbE6C6ylM6zZS9p?usp=sharing) | [***RISE*-2** Policy](https://github.com/rise-policy/RISE-2)

**Authors**: [Hongjie Fang*](https://tonyfang.net/), [Chenxi Wang*](https://github.com/chenxi-wang), [Yiming Wang*](mailto:sommerfeld@sjtu.edu.cn), [Jingjing Chen*](https://github.com/junxix/), [Shangning Xia](https://github.com/Xiashangning), [Jun Lv](https://lyuj1998.github.io/), [Zihao He](https://github.com/Alan-Heoooh), [Xiyan Yi](mailto:simonyxy@sjtu.edu.cn), [Yunhan Guo](mailto:yunhan_guo@sjtu.edu.cn), [Xinyu Zhan](https://github.com/kelvin34501), [Lixin Yang](https://lixiny.github.io/), [Weiming Wang](mailto:wangweiming@sjtu.edu.cn), [Cewu Lu](https://www.mvig.org/), [Hao-Shu Fang](https://fang-haoshu.github.io/)   (* = Equal Contribution)

This repository contains the full source codes of the ***AirExo*-2** project. This codebase is also compatible with [the original ***AirExo***](https://airexo.tech/airexo1/).

## 🔥 News

- **[Jun. 1, 2025]** Update sample data collected by ***AirExo*-2**.
- **[May 15, 2025]** Initial release.

## Table of Contents

- [🛠️ Installation](#️-installation)
- [📷 Calibration](#-calibration)
- [🎮 Data Collection: Teleoperation](#️-data-collection-1-teleoperation)
- [🛢️ Data Collection: In-the-Wild](#️-data-collection-2-in-the-wild)
- [🤖 Policy Training and Evaluations](#-policy-training-and-evaluations)
- [🕵️ Testing Scripts](#️-testing-scripts)

## 🛠️ Installation

### 🔩 Exoskeletons

Please refer to [the installation guide](https://airexo.tech/airexo2/static/pdfs/installation-guide.pdf) for ***AirExo*-2** assembly.

### 🤖 Real Robot

We use two [Flexiv Rizon 4](https://www.flexiv.com/product/rizon) robotics arms, both equipped with the [Robotiq 2F-85 grippers](https://robotiq.com/products/adaptive-grippers), as our dual-arm robot platform. The right arm is the mirrored version of the left arm. Both arms are mounted on the base at a 45 degree angle *w.r.t.* the ground. An Intel RealSense D415 camera is mounted on the top of the workspace to capture global observations. Two Intel RealSense D415 cameras are mounted on the wrist as in-hand cameras to capture in-hand observations. The in-hand observations are only used for 2D image-based policies when they are trained with teleoperated demonstrations.

### 💻 Conda Environments

Create the `airexo` conda environment and install necessary packages.

```bash
conda create -n airexo python=3.8
conda activate airexo
pip install -r requirements.txt
```

### 📁 Dependencies

Follow [the dependency installation instructions](assets/docs/DEPENDENCIES.md) to install dependencies.

## 🔨 Configuration

Please follow [the configuration guide](assets/docs/CONFIG.md) to configurate: (1) the ***AirExo*-2** joint configurations; (2) the robot joint configurations; (3) the camera configurations.

## 📷 Calibration

Please follow [the calibration guide](assets/docs/CALIB.md) to calibrate: (1) the joint configurations of ***AirExo*-2**, and (2) the transformation between ***AirExo*-2** and the camera.

## 👀 Visualization

We recommend performing visualization before data collection to ensure accurate calibration. Please follow [the visualization guide](assets/docs/VIS.md) for more details.

## 🎮 Data Collection (1): Teleoperation

### 🗂️ Demonstration Collection

Although the primary goal of ***AirExo*-2** is to facilitate large-scale in-the-wild demonstration collection, we have also implemented teleoperation-based data collection in this codebase for compatibility.  

Follow the [teleoperation data collection configuration guide](assets/docs/CONFIG.md#-teleoperation) to set up your configurations. Ensure that the `type` in the configuration file is set to `teleop`. Then, modify the camera collection script [`airexo/collection/camera_collector.sh`](airexo/collection/camera_collector.sh) to fit your environment (*e.g.*, update the conda path).  

Once the setup is complete, use the following command to start the teleoperation process for data collection:  

```bash
python -m airexo.collection.main --config-name /path/to/your/teleop/config
```

At the start of the data collection process, the program will prompt you to enter a **task ID** (an integer up to four digits) to represent the task. Next, you will be asked to enter a **scene ID** (also an integer up to four digits). After confirming these IDs, the dual-arm robot will initialize to a predefined fixed position.  

Once initialization is complete, the robot will align its joints with ***AirExo*-2**, meaning both arms will gradually move to match the current ***AirExo*-2** joint configuration. During this process, keep the exoskeleton arms as still as possible to prevent sudden robotic arm movements. Once the alignment between the robot and ***AirExo*-2** is complete, the teleoperation process will start automatically.  

To begin collecting demonstration data, press **`r`** to start the collection program. Wait for all processes to initialize in the command line, then the operator can perform the task via teleoperation. After completing the task, press **`q`** to stop data collection. Notice that this only stops data collection, but the operator remains in control of the robot via ***AirExo*-2**.

After finishing a demonstration, the system will ask whether to continue collecting data:  
- To end data collection, enter `0` and press **Enter**. The teleoperation process will stop, and the robot will return to its initial position.  
- To continue, enter `1` and press **Enter**. The system will automatically increment the scene ID and start the next demonstration.  

### ➡️ (Optional) Dataset Transformation

After collecting teleoperation demonstrations, they can be directly used for downstream policy training. 

For [***RISE*-2** policy codebase](http://github.com/rise-policy/RISE-2/), an additional transformation is needed to convert the dataset into the required calibration format. Please refer to [the dataset transformation guide](assets/docs/TRANSFORM.md) for more details.

## 🛢️ Data Collection (2): In-the-Wild

### 🗂️ Demonstration Collection

Follow the [in-the-wild data collection configuration guide](assets/docs/CONFIG.md#️-in-the-wild) to set up your configurations. Ensure that the `type` in the configuration file is set to `wild`. Then, modify the camera collection script [`airexo/collection/camera_collector.sh`](airexo/collection/camera_collector.sh) to fit your environment (*e.g.*, update the conda path).  

```bash
python -m airexo.collection.main --config-name /path/to/your/wild/config
```

The data collection process follows the same steps as the teleoperation data collection, with one key difference: instead of teleoperating the robot to complete the task, the operator directly performs the task using ***AirExo*-2**.

### 🔃 Demonstration Adaptor Guide

After collecting in-the-wild demonstrations, we need to convert them into pseudo-robot demonstrations using the demonstration adaptor. This adaptation process significantly improves the performance of downstream policies trained on these demonstrations.

Please follow [the demonstration adaptor guide](assets/docs/ADAPTOR.md) to transform the in-the-wild demonstrations into the pseudo-robot demonstrations.

### ➡️ Dataset Transformation

After in-the-wild demonstration collection, additional dataset transformation is required to convert the robot actions into the camera coordinate frame (operation space adaptor) and adapt the dataset for downstream ***RISE*-2** training. 

Please refer to [the dataset transformation guide](assets/docs/TRANSFORM.md) for more details.

## 💾 Data Format

The raw data after collection has the following format:

```
task_0001
|-- scene_0001
|   |-- meta.json                  # metadata
|   |-- cam_[serial_number 1]/    
|   |   |-- color                  # RGB
|   |   |   |-- [timestamp 1].png
|   |   |   |-- [timestamp 2].png
|   |   |   |-- ...
|   |   |   `-- [timestamp T].png
|   |   `-- depth                  # depth
|   |       |-- [timestamp 1].png
|   |       |-- [timestamp 2].png
|   |       |-- ...
|   |       `-- [timestamp T].png
|   |-- cam_[serial_number 2]/     # similar camera structure
|   `-- lowdim/                    # low-dimensional data, with a relative high frequency.
|       |-- robot_left.h5          # left robot information [only for teleop]
|       |-- robot_right.h5         # right robot information [only for teleop]
|       |-- gripper_left.h5        # left gripper information [only for teleop]
|       |-- gripper_right.h5       # right gripper information [only for teleop]
|       |-- airexo_left.h5         # left AirExo-2 information [only for wild]
|       `-- airexo_right.h5        # right AirExo-2 information [only for wild]
|-- scene_0002                     # similar scene structure
|-- ...
`-- scene_0050                     # similar scene structure
```

The dataset transformation (1) aligns the high-frequency low-dim data with the camera timestamp; (2) use the operation space adaptor to transform the in-the-wild robot action into the camera coordinate; (3) transform the calibration results into suitable format and add its timestamp into the metadata; and (4) re-organize the directory for downstream policy training. After dataset transformation, the data has the following format:

```
task_0001
|-- calib
|   |-- [calib_timestamp].npy      # calibration information (in a new format)
|-- scene_0001
|   |-- meta.json                  # metadata (with calib_timestamp)
|   |-- cam_[serial_number 1]/    
|   |   |-- color                  # RGB
|   |   |   |-- [timestamp 1].png
|   |   |   |-- [timestamp 2].png
|   |   |   |-- ...
|   |   |   `-- [timestamp T].png
|   |   `-- depth                  # depth
|   |       |-- [timestamp 1].png
|   |       |-- [timestamp 2].png
|   |       |-- ...
|   |       `-- [timestamp T].png
|   |-- cam_[serial_number 2]/     # similar camera structure
|   `-- lowdim/                    # low-dimensional data, with a relative high frequency.
|       |-- robot_left.h5          # left robot information [only for teleop]
|       |-- robot_right.h5         # right robot information [only for teleop]
|       |-- gripper_left.h5        # left gripper information [only for teleop]
|       |-- gripper_right.h5       # right gripper information [only for teleop]
|       |-- airexo_left.h5         # left AirExo-2 information [only for wild]
|       |-- airexo_right.h5        # right AirExo-2 information [only for wild]
|       |-- [timestamp 1].npy      # tcp pose, gripper information and gripper action, align with the camera timestamp 1
|       |-- [timestamp 2].npy      # tcp pose, gripper information and gripper action, align with the camera timestamp 2
|       |-- ...
|       `-- [timestamp T].npy      # tcp pose, gripper information and gripper action, align with the camera timestamp T
|-- scene_0002                     # similar scene structure
|-- ...
`-- scene_0050                     # similar scene structure
```

Dataset with this format can be directly used for the downstream ***RISE*-2** policy training. Please refer to [the policy training and evaluation guide](#-policy-training-and-evaluations) for more details.

## 🤖 Policy Training and Evaluations

Please follow the instructions in [the ***RISE*-2** repository](https://github.com/rise-policy/RISE-2) to train the ***RISE*-2** policy and evaluate it on the real robot platform. 

## 🕵️ Testing Scripts

Several testing scripts are also provided in this codebase to facilitate modular testing. Please refer to [the testing scripts explanations](assets/docs/TESTS.md) for more details.

## 🙏 Acknowledgement

- Our data collection module is built upon [the data collection script of the original AirExo](https://github.com/AirExo/collector), with the original MIT license.
- Our calibration code is built upon [redner](https://github.com/BachiLi/redner).
- Our segmentation code is adapted from [SAM2](https://github.com/facebookresearch/sam2/). This part is under Apache-2.0 license.
- Our inpainting code is adapted from [ProPainter](https://github.com/sczhou/ProPainter). This part is under S-Lab license.
- Our conditional image generation code is adapted from [ControlNet](https://github.com/lllyasviel/ControlNet). This part is under Apache-2.0 license.

## ✍️ Citation

```bibtex
@article{fang2025airexo,
  title   = {AirExo-2: Scaling up Generalizable Robotic Imitation Learning with Low-Cost Exoskeletons},
  author  = {Hongjie Fang and Chenxi Wang and Yiming Wang and Jingjing Chen and Shangning Xia and Jun Lv and Zihao He and Xiyan Yi and Yunhan Guo and Xinyu Zhan and Lixin Yang and Weiming Wang and Cewu Lu and Hao-Shu Fang},
  journal = {arXiv preprint arXiv:2503.03081},
  year    = {2025}
}

@inproceedings{fang2024airexo,
  title =        {AirExo: Low-Cost Exoskeletons for Learning Whole-Arm Manipulation in the Wild},
  author =       {Fang, Hongjie and Fang, Hao-Shu and Wang, Yiming and Ren, Jieji and Chen, Jingjing and Zhang, Ruo and Wang, Weiming and Lu, Cewu},
  booktitle =    {2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages =        {15031--15038},
  year =         {2024},
  organization = {IEEE}
}
```

## ☎️ Contact

Project Curator: [Hongjie Fang](mailto:galaxies@sjtu.edu.cn).

Project Core Contributor: [Chenxi Wang](mailto:cxwang85@gmail.com), [Yiming Wang](mailto:sommerfeld@sjtu.edu.cn) and [Jingjing Chen](mailto:jjchen20@sjtu.edu.cn).

Project Advisors: [Cewu Lu](mailto:lucewu@sjtu.edu.cn) and [Hao-Shu Fang](mailto:fhaoshu@gmail.com).

## 📃 License

***AirExo*-2** (including hardware models, codebase and data) by [the AirExo Team](http://airexo.tech/#team) is licensed under <a href="https://creativecommons.org/licenses/by-nc-sa/4.0/?ref=chooser-v1" target="_blank" rel="license noopener noreferrer" style="display:inline-block;">CC BY-NC-SA 4.0<img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/cc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/by.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/nc.svg?ref=chooser-v1" alt=""><img style="height:22px!important;margin-left:3px;vertical-align:text-bottom;" src="https://mirrors.creativecommons.org/presskit/icons/sa.svg?ref=chooser-v1" alt=""></a></p>