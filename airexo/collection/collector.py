"""
Collect lowdim data.
"""
import os
import h5py
import time
import logging
import threading

from airexo.helpers.logger import ColoredLogger


class LowdimCollector:
    def __init__(self, name, device, freq = 100, buffer_size = 1000, logger_name = "Lowdim Collector", **kwargs):
        logging.setLoggerClass(ColoredLogger)
        self.logger = logging.getLogger(logger_name)
        self.name = name
        self.is_collect = False
        self.data_path = None
        self.count = 0
        self.device = device
        self.freq = freq
        self.interval = 1.0 / freq
        self.buffer_size = buffer_size

        self.fields = []
        device_states = device.get_states()
        for key, value in device_states.items():
            self.fields.append({
                "name": key,
                "shape": (0, ) + value.shape,
                "maxshape": (None, ) + value.shape,
                "chunks": (buffer_size, ) + value.shape,
                "dtype": str(value.dtype)
            })
        
        self.thread = threading.Thread(target = self._thread)
        self.thread.setDaemon(True)
        self.thread.start()
        
    def _thread(self):
        while True:
            tic = time.time()
            if self.is_collect:
                states = self.device.get_states()
                timestamp = int(time.time() * 1000)
                self.timestamp.resize((self.count + 1, ))
                self.timestamp[-1] = timestamp
                for name in self.dataset.keys():
                    self.dataset[name].resize((self.count + 1, ) + self.dataset[name].shape[1:])
                    self.dataset[name][-1] = states[name]
                self.count += 1
            duration = time.time() - tic
            if duration < self.interval:
                time.sleep(self.interval - duration)

    def start_collect(self, data_path):
        if self.is_collect:
            raise RuntimeError("Please close current collection before starting new collection.")
        
        self.data_path = data_path
        self.count = 0

        self.file = h5py.File(os.path.join(data_path, "{}.h5".format(self.name)), "w")
        self.timestamp = self.file.create_dataset("timestamp", shape = (0, ), maxshape = (None, ), chunks = (self.buffer_size, ), dtype = "int64")
        
        self.dataset = {}
        for field in self.fields:
            self.dataset[field["name"]] = self.file.create_dataset(**field)
        
        self.is_collect = True
        self.logger.info("Start collecting.")
    
    def stop_collect(self):
        if self.is_collect:
            self.is_collect = False
            time.sleep(5 * self.interval)
            self.file.close()
            self.logger.info("Stop.")
    
    def stop(self):
        self.stop_collect()
        self.thread.join()
