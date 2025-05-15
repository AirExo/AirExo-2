import numpy as np

from omegaconf import OmegaConf
from airexo.device.arm import FlexivArm
from airexo.helpers import degree as utils_deg
from airexo.device.gripper import Robotiq2F85Gripper


class Robot(FlexivArm):
    def __init__(
        self,
        serial,
        joint_cfgs,
        gripper: Robotiq2F85Gripper,
        logger_name: str = "Flexiv Robot",
        min_joint_diff = 0.01,
        **kwargs
    ):
        self.joint_cfgs = OmegaConf.create(joint_cfgs)
        self.gripper = gripper
        super(Robot, self).__init__(
            serial = serial,
            logger_name = logger_name,
            **kwargs
        )
        # TODO: test whether necessary
        self.prev_action = [0.0] * self.joint_cfgs.num_joints
        self.min_joint_diff = min_joint_diff

    def init_joint(
        self,
        max_vel,
        max_acc
    ):
        target_vel = [0.0] * self.joint_cfgs.num_robot_joints
        target_acc = [0.0] * self.joint_cfgs.num_robot_joints

        init_joint_pos = []
        for joint_id in range(1, self.joint_cfgs.num_joints + 1):
            joint_name = "joint{}".format(joint_id)
            value = self.joint_cfgs[joint_name].init_value
            if "zero_centered" in self.joint_cfgs[joint_name].keys() and self.joint_cfgs[joint_name].zero_centered:
                value = utils_deg.deg_zero_centered(value, self.joint_cfgs[joint_name].min, self.joint_cfgs[joint_name].max, self.joint_cfgs[joint_name].direction)
            if "rad" in self.joint_cfgs[joint_name].keys() and self.joint_cfgs[joint_name].rad:
                value = utils_deg.deg_2_rad(value)
            init_joint_pos.append(value)
        init_joint_pos = np.array(init_joint_pos)

        self.prev_pos = init_joint_pos

        self.send_joint_pos(
            init_joint_pos[:self.joint_cfgs.num_robot_joints],
            max_vel, 
            max_acc,
            target_vel,
            target_acc,
            impedance = False,
            blocking = True
        )
        self.gripper.set_width(*init_joint_pos[self.joint_cfgs.num_robot_joints:])


    def action_joint(
        self, 
        action, 
        max_vel,
        max_acc,
        impedance = True,
        wait = False
    ):
        target_vel = [0.0] * self.joint_cfgs.num_robot_joints
        target_acc = [0.0] * self.joint_cfgs.num_robot_joints
        
        if all(abs(self.prev_action[i] - action[i]) <= self.min_joint_diff for i in range(self.joint_cfgs.num_robot_joints)):
            action[:self.joint_cfgs.num_robot_joints] = self.prev_action[:self.joint_cfgs.num_robot_joints]
        
        for joint_id in range(1, self.joint_cfgs.num_joints + 1):
            joint_name = "joint{}".format(joint_id)
            if self.joint_cfgs[joint_name].fixed:
                value = self.joint_cfgs[joint_name].fixed_value
                if "zero_centered" in self.joint_cfgs[joint_name].keys() and self.joint_cfgs[joint_name].zero_centered:
                    value = utils_deg.deg_zero_centered(value, self.joint_cfgs[joint_name].min, self.joint_cfgs[joint_name].max, self.joint_cfgs[joint_name].direction)
                if "rad" in self.joint_cfgs[joint_name].keys() and self.joint_cfgs[joint_name].rad:
                    value = utils_deg.deg_2_rad(value)
                action[joint_id - 1] = value

        self.prev_action = action

        self.send_joint_pos(
            action[:self.joint_cfgs.num_robot_joints],
            max_vel, 
            max_acc,
            target_vel,
            target_acc,
            impedance = impedance,
            blocking = wait
        )

        self.gripper.set_width(*action[self.joint_cfgs.num_robot_joints:])
