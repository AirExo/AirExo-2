"""
Shared Memory Manager.

Author: Hongjie Fang.
"""

import numpy as np
from multiprocessing import shared_memory


class SharedMemoryManager(object):
    """
    Shared Memory Manager.
    """
    def __init__(self, name, type = 0, shape = (1,), dtype = np.float32):
        """
        Initialization.
        
        Parameters
        ----------
        - name: the name of the shared memory;
        - type: integer in [0, 1];
            * 0: sender;
            * 1: receiver.
        - shape: optional, default: (1,), the array shape.
        - dtype: optional, default: np.float32, the element type of the array.
        """
        super(SharedMemoryManager, self).__init__()
        self.name = name
        self.type = type
        self.shape = shape
        if isinstance(dtype, str):
            dtype = to_dtype(dtype)
        self.dtype = np.dtype(dtype)
        if self.type not in [0, 1]:
            raise AttributeError('Invalid type in shared memory manager.')
        if self.type == 0:
            self.shared_memory = shared_memory.SharedMemory(name = self.name, create = True, size = self.dtype.itemsize * np.prod(self.shape))
            self.buf = np.ndarray(self.shape, dtype = self.dtype, buffer = self.shared_memory.buf)
        else:
            self.shared_memory = shared_memory.SharedMemory(name = self.name)
        
    def execute(self, arr = None):
        """
        Execute the function.

        Paramters
        ---------
        - arr: np.array object, only used in sender, the array.
        """
        if self.type == 0:
            if arr is None:
                raise AttributeError('Array should be specified in shared memory sender.')
            try:
                self.buf[:] = arr[:]
            except Exception:
                raise AttributeError('Size mismatch in shared memory receiver.')
        else:
            ret_arr = np.copy(np.ndarray(self.shape, dtype = self.dtype, buffer = self.shared_memory.buf))
            return ret_arr

    def close(self):
        self.shared_memory.close()
        if self.type == 0:
            self.shared_memory.unlink()

def to_dtype(s):
    if s == "bool":
        return bool
    else:
        return getattr(np, s)
