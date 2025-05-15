import os
import sys
import time
import hydra
import termios
import numpy as np

from pynput import keyboard

from airexo.helpers.logger import setup_loggers
from airexo.helpers.shared_memory import SharedMemoryManager
from airexo.helpers.collection import collection_process_cleanup


@hydra.main(
    version_base = None,
    config_path = os.path.join("..", "configs", "collection")
)
def main(cfg):
    setup_loggers()
    collection_process_cleanup()

    controller = hydra.utils.instantiate(cfg.controller)

    tid = int(input('Task ID: '))
    sid = int(input('Scene ID: '))

    controller.initialize()
    controller.start()

    lowdim_collectors = []
    for lc_config in cfg.lowdim_collectors:
        lc_object = controller
        for key in lc_config["controller_key"]:
            lc_object = getattr(lc_object, key)
        lowdim_collectors.append(hydra.utils.instantiate(lc_config, device = lc_object))

    has_record = False
    has_stop = False
    shm_sender = SharedMemoryManager("record_flag", 0, shape = (2, ), dtype = np.bool_)
    shm_sender.execute(np.array([False, False], dtype = np.bool_))


    while True:
        data_path = os.path.join(cfg.path, "task_{:04d}".format(tid), "scene_{:04d}".format(sid))
        lowdim_path = os.path.join(data_path, "lowdim")
        os.makedirs(data_path, exist_ok = True)
        os.makedirs(lowdim_path, exist_ok = True)

        def _on_press(key):
            nonlocal has_record, has_stop
            try:
                if key.char == 'q':
                    has_stop = True  
                    shm_sender.execute(np.array([has_record, has_stop], dtype = np.bool_))
                    for lowdim_collector in lowdim_collectors:
                        lowdim_collector.stop_collect()
                if key.char == 'r':
                    if not has_record:
                        for cam_serial in cfg.cameras:
                            os.system('bash airexo/collection/camera_collector.sh {} {} {} &'.format(cam_serial, data_path, cfg.camera_freq))
                        for lowdim_collector in lowdim_collectors:
                            lowdim_collector.start_collect(lowdim_path)
                        has_record = True
                        shm_sender.execute(np.array([has_record, has_stop], dtype = np.bool_))
                    else:
                        pass     
            except Exception as e:
                # TODO: make it elegant to ignore other keys
                pass

        def _on_release(key):
            pass

        listener = keyboard.Listener(on_press = _on_press, on_release = _on_release)
        listener.start()

        while True:
            if has_stop:
                break
            time.sleep(0.05)

        listener.stop()

        try:
            time.sleep(1)
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
            option = int(input("Next: task {}, scene {}, continue? (1/0)".format(tid, sid + 1)))
            if option == 0:
                break
        except Exception as e:
            print(e)
            break
        
        sid += 1
        has_record = False
        has_stop = False
        shm_sender.execute(np.array([False, False], dtype = np.bool_))

    time.sleep(1)
    controller.stop()
    time.sleep(1)
    shm_sender.close()

    collection_process_cleanup()


if __name__ == '__main__':
    main()