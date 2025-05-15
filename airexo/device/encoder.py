'''
Angle Encoder Interface.

Author: Hongjie Fang.
'''

import copy
import time
import serial
import logging
import numpy as np

from airexo.helpers.logger import ColoredLogger


def hex2dex(e_hex):
    return int(e_hex, 16)

def hex2bin(e_hex):
    return bin(int(e_hex, 16))

def dex2bin(e_dex):
    return bin(e_dex)

def crc16(hex_num):
    """
    CRC16 verification
    :param hex_num:
    :return:
    """
    crc = '0xffff'
    crc16 = '0xA001'
    test = hex_num.split(' ')

    crc = hex2dex(crc)  
    crc16 = hex2dex(crc16) 
    for i in test:
        temp = '0x' + i
        temp = hex2dex(temp) 
        crc ^= temp  
        for i in range(8):
            if dex2bin(crc)[-1] == '0':
                crc >>= 1
            elif dex2bin(crc)[-1] == '1':
                crc >>= 1
                crc ^= crc16

    crc = hex(crc)
    crc_H = crc[2:4]
    crc_L = crc[-2:]

    return crc, crc_H, crc_L


class AngleEncoder:
    """
    Angle Encoder(s) Interface: receive signals from the angle encoders.
    """
    def __init__(
        self,
        ids,
        port,
        baudrate = 115200,
        sleep_gap = 0.001,
        logger_name = "Angle Encoder",
        **kwargs 
    ) -> None:
        """
        Args:
        - ids: list of int, e.g., [1, 2, ..., 8], the id(s) of the desired encoder(s);
        - port, baudrate, (**kwargs): the args of the serial agents;
        - sleep_gap: float, optional, default: 0.001, the sleep gap between adjacent write options;
        - logger_name: str, optional, default: "AngleEncoder", the name of the logger.
        """
        logging.setLoggerClass(ColoredLogger)
        self.logger = logging.getLogger(logger_name)

        self.ids = ids
        self.ids_num = len(ids)
        self.ids_map = {}
        for i, id in enumerate(ids):
            self.ids_map[id] = i
        self.sleep_gap = sleep_gap
        self.ser = serial.Serial(port, baudrate = baudrate, **kwargs)
        if not self.ser.is_open:
            raise RuntimeError('Fail to open the serial port, please check your settings again.')
        self.ser.flushInput()
        self.ser.flushOutput()
        self.last_angle = None

    def get_angle(self, ignore_error: bool = False):
        """
        Get the angle of the encoder.

        Args:
        - ignore_error: bool, optional, default: False, whether to ignore the incomplete data error (if set True, then the last results will be used.)
        
        Returns:
        - ret: np.array, the encoder angle results corresponding to the ids array.
        """
        self.ser.flushInput()
        ids = copy.deepcopy(self.ids)
        for i in ids:
            sendbytes = str(i).zfill(2) + " 03 00 41 00 01"
            crc, crc_H, crc_L = crc16(sendbytes)
            sendbytes = sendbytes + ' ' + crc_L + ' ' + crc_H
            sendbytes = bytes.fromhex(sendbytes)
            self.ser.write(sendbytes) 
            time.sleep(self.sleep_gap)
        
        re = self.ser.read(len(ids) * 7)
        if self.ser.inWaiting() > 0:
            se = self.ser.read_all()
            re += se
        
        count = 0
        remains = ids.copy()
        if ignore_error:
            ret = np.copy(self.last_angle).astype(np.float32)
        else:
            ret = np.zeros(self.ids_num).astype(np.float32)
        b = 0
        while b <= len(re) - 7:
            if re[b + 1] == 3 and re[b + 2] == 2 and re[b] in remains:
                angle = 360 * (re[b + 3] * 256 + re[b + 4]) / 4096
                ret[self.ids_map[re[b]]] = angle
                count += 1
                remains.remove(re[b])
                b += 7
            else:
                b += 1
        if not ignore_error and count != len(ids):
            raise RuntimeError('Failure to receive all encoders, errors occurred in ID {}.'.format(remains))
        self.last_angle = ret
        return ret
    
    def stop(self):
        self.ser.close()
