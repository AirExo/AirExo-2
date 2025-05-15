'''
Robotiq Gripper Interface.

Author: Anonymous, Hongjie Fang.

Refererences:
  [1] https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf
'''

import time
import struct
import serial
import logging
import threading
import numpy as np

from airexo.helpers.logger import ColoredLogger


class Robotiq2F85Gripper:
    '''
    Robotiq Gripper 2F-85 Gripper API.
    '''
    def __init__(
        self, 
        port: str, 
        logger_name: str = "Dahuan Gripper",
        **kwargs
    ) -> None:
        '''
        Initialization.
        
        Parameters:
        - port: str, the port of the gripper;
        - logger_name: str, optional, default: "Device", the name of the logger.
        '''
        logging.setLoggerClass(ColoredLogger)
        self.logger = logging.getLogger(logger_name)

        self.port = port
        self.waiting_gap = 0.02
        self.max_width = 0.085
        self.ser = serial.Serial(port = self.port, baudrate = 115200, timeout = 1, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS)
        self.lock = threading.Lock()
        self.activate()
        self.last_width = 0
        self.close_gripper()
    
    def activate(self):
        '''
        Activate the gripper.
        Refer to: page 62 of ref [1].
        '''
        # Activation Request
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
        response = self.ser.read(8)
        if response != b"\x09\x10\x03\xE8\x00\x03\x01\x30":
            raise AssertionError('Unexpected response of the gripper.')
        time.sleep(self.waiting_gap)
        self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00\x72\xE1")
        response = self.ser.read(8)
        if response != b"\x09\x10\x03\xE8\x00\x03\x01\x30":
            raise AssertionError('Unexpected response of the gripper.')
        time.sleep(self.waiting_gap)
        # Read Gripper status until the activation is completed
        self.ser.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
        while self.ser.read(7) != b"\x09\x03\x02\x31\x00\x4C\x15":
            time.sleep(self.waiting_gap)
            self.ser.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")

    def _transform_width(self, width, to_control: bool = False):
        '''
        Transform gripper width to real-world width in meters.
        - to_control = True: gripper width in meters to gripper width signals.
        - to_control = False: gripper width signals to gripper width in meters.
        '''
        if to_control:
            width = int(width / self.max_width * 255)
            width = 255 - np.clip(width, 0, 255)
        else:
            width = 255 - np.clip(width, 0, 255)
            width = width / 255. * self.max_width
        return width

    def set_width(self, width: float, speed: int = 255, force: int = 77):
        '''
        Set gripper width.
        '''
        width_signal = self._transform_width(width, to_control = True)
        speed = int(np.clip(speed, 0, 255))
        force = int(np.clip(force, 0, 255))
        command = bytearray(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\x00\x00")
        command[10] = width_signal
        command[11] = speed
        command[12] = force
        with self.lock:
            self.send_command(command)
            self.ser.read(8)
        self.last_width = np.clip(width, 0, self.max_width)

    def open_gripper(self):
        '''
        Open the gripper at full speed and full force.
        Refer to: page 68 of ref [1].
        '''
        with self.lock:
            self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
            response = self.ser.read(8)
            if response != b"\x09\x10\x03\xE8\x00\x03\x01\x30":
                raise AssertionError('Unexpected response of the gripper.')

    def close_gripper(self):
        '''
        Close the gripper at full speed and full force.
        Refer to: page 65 of ref [1].
        '''
        with self.lock:
            self.ser.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\x64\x03\x82")
            response = self.ser.read(8)
            if response != b"\x09\x10\x03\xE8\x00\x03\x01\x30":
                raise AssertionError('Unexpected response of the gripper.')

    def send_command(self, command) -> None:
        crc = self._calc_crc(command)
        data = command + crc
        self.ser.write((data))

    def get_states(self) -> None:
        '''
        Update the current information about the gripper (width, last width).
        Refer to: page 66-67, 69-70 of ref [1].
        ''' 
        with self.lock:
            self.ser.write(b"\x09\x03\x07\xD0\x00\x03\x04\x0E")
            data = self.ser.read(11)
        # TODO: 
        # if data[:3] != b"\x09\x03\x06":
        #     return None
        return {
            "width": np.array(self._transform_width(data[7]), dtype = np.float32), 
            "action": np.array(self.last_width, dtype = np.float32)
        }

    def get_width(self):
        with self.lock:
            self.ser.write(b"\x09\x03\x07\xD0\x00\x03\x04\x0E")
            data = self.ser.read(11)
        # TODO: 
        # if data[:3] != b"\x09\x03\x06":
        #     return None
        return self._transform_width(data[7])

    def get_last_width(self):
        return self.last_width

    def _calc_crc(self, command: bytearray) -> bytearray:
        '''
        Calculate the Cyclic Redundancy Check (CRC) bytes for command.

        Parameters:
        - command: bytes, required, the given command.
        
        Returns:
        - The calculated CRC bytes.
        '''
        crc_registor = 0xFFFF
        for data_byte in command:
            tmp = crc_registor ^ data_byte
            for _ in range(8):
                if(tmp & 1 == 1):
                    tmp = tmp >> 1
                    tmp = 0xA001 ^ tmp
                else:
                    tmp = tmp >> 1
            crc_registor = tmp
        crc = bytearray(struct.pack('<H', crc_registor))
        return crc

    def stop(self):
        self.ser.close()