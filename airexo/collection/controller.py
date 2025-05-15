import time
import logging
import threading
import numpy as np

from omegaconf import OmegaConf

from airexo.device.robot import Robot
from airexo.device.airexo import AirExo
from airexo.helpers.logger import ColoredLogger
from airexo.helpers.transform import transform_arm


class SingleArmTeleoperator:
    """
    Teleoperator for single robot arm.
    """
    def __init__(
        self,
        robot: Robot,
        airexo: AirExo,
        calib_cfgs,
        logger_name: str = "TeleOP",
        max_vel_safe = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3],
        max_acc_safe = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        max_vel_rt = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        max_acc_rt = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        use_impedance: bool = True,
        impedance_joint_stiffness = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
        impedance_joint_damping_ratio = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
        **kwargs
    ):
        logging.setLoggerClass(ColoredLogger)
        self.logger = logging.getLogger(logger_name)
        self.robot = robot
        self.airexo = airexo
        self.max_vel_safe = max_vel_safe
        self.max_acc_safe = max_acc_safe
        self.max_vel_rt = max_vel_rt
        self.max_acc_rt = max_acc_rt
        self.use_impedance = use_impedance
        self.impedance_joint_stiffness = impedance_joint_stiffness
        self.impedance_joint_damping_ratio = impedance_joint_damping_ratio
        assert robot.joint_cfgs.num_joints == airexo.joint_cfgs.num_joints
        self.calib_cfgs = OmegaConf.create(calib_cfgs)
    
    def transform(self, data):
        """
        Transform the AirExo action into the robot action.
        """
        return transform_arm(
            robot_cfgs = self.robot.joint_cfgs,
            airexo_cfgs = self.airexo.joint_cfgs,
            calib_cfgs = self.calib_cfgs,
            data = data
        )

    def initialize(self):
        """
        Initialize the robot.
        """
        self.logger.info('Initialize...')
        self.robot.init_joint(
            max_vel = self.max_vel_safe,
            max_acc = self.max_acc_safe
        )
        self.logger.info('Calibrate ... Please remain still.')
        airexo_res = self.airexo.get_angle()
        robot_res = self.transform(airexo_res)
        self.robot.action_joint(
            robot_res, 
            max_vel = self.max_vel_safe,
            max_acc = self.max_acc_safe,
            impedance = False,
            wait = True
        )
        self.logger.info('Finish initialization.')

    def start(self, delay_time = 0.0):
        """
        Start teleoperation.
        
        Parameters:
        - delay_time: float, optional, default: 0.0, the delay time before collecting data.
        """
        self.thread = threading.Thread(target = self.teleop_thread, kwargs = {'delay_time': delay_time})
        self.thread.setDaemon(True)
        self.thread.start()
    
    def teleop_thread(self, delay_time = 0.0):
        time.sleep(delay_time)
        self.is_teleop = True
        self.logger.info('Start teleoperation ...')
        # Set joint impedance before teleoperation
        if self.use_impedance:
            self.robot.set_joint_impedance(self.impedance_joint_stiffness, self.impedance_joint_damping_ratio)
        # Start teleoperation
        while self.is_teleop:
            airexo_res = self.airexo.get_angle()
            robot_res = self.transform(airexo_res)
            self.robot.action_joint(
                robot_res,
                max_vel = self.max_vel_rt,
                max_acc = self.max_acc_rt,
                impedance = self.use_impedance,
                wait = False
            )
    
    def stop(self):
        """
        Stop teleoperation process.
        """
        self.is_teleop = False
        if self.thread:
            self.thread.join()
        self.logger.info('Stop teleoperation.')
        self.airexo.stop()
        self.robot.init_joint(
            max_vel = self.max_vel_safe,
            max_acc = self.max_acc_safe
        )
        self.robot.stop()
        self.robot.gripper.stop()


class SingleArmDummyController:
    """
    Dummy controller for in-the-wild demonstration collection.
    """
    def __init__(
        self,
        airexo: AirExo,
        logger_name: str = "DummyCtrl",
        **kwargs
    ):
        logging.setLoggerClass(ColoredLogger)
        self.logger = logging.getLogger(logger_name)
        self.airexo = airexo

    def initialize(self):
        """
        Initialize.
        """
        self.logger.info("Initialization finished.")

    def start(self, delay_time = 0.0):
        """
        Start.
        """
        pass
    
    def stop(self):
        """
        Stop.
        """
        self.logger.info("Stop")
        self.airexo.stop()
 

class DualArmController:
    """
    Controller for dual robot arm.
    """
    def __init__(
        self,
        left_arm,
        right_arm,
        **kwargs
    ):
        self.left_arm = left_arm
        self.right_arm = right_arm
    
    def initialize(self):
        """
        Initialize the robot.
        """
        thread_left = threading.Thread(target = lambda: self.left_arm.initialize())
        thread_right = threading.Thread(target = lambda: self.right_arm.initialize())
        thread_left.start()
        thread_right.start()
        thread_left.join()
        thread_right.join()
    
    def start(self, delay_time = 0.0):
        """
        Start teleoperation.
        
        Parameters:
        - delay_time: float, optional, default: 0.0, the delay time before collecting data.
        """
        self.left_arm.start(delay_time = delay_time)
        self.right_arm.start(delay_time = delay_time)
    
    def stop(self):
        """
        Stop the teleoperation process.
        """
        thread_left = threading.Thread(target = lambda: self.left_arm.stop())
        thread_right = threading.Thread(target = lambda: self.right_arm.stop())
        thread_left.start()
        thread_right.start()
        thread_left.join()
        thread_right.join()
    