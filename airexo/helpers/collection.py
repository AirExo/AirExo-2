"""
Data Collection Utilities.

Authors: Hongjie Fang.
"""

import os


def collection_process_cleanup():
    os.system("kill -9 `ps -ef | grep collector | grep -v grep | awk '{print $2}'`")
    os.system("kill -9 `ps -ef | grep camera | grep -v grep | awk '{print $2}'`")
    os.system('rm -f /dev/shm/*')