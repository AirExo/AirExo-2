"""
Loggers.
"""

import logging
from colorlog import ColoredFormatter


class ColoredLogger(logging.Logger):
    def __init__(self, name):
        super(ColoredLogger, self).__init__(name)
        formatter = ColoredFormatter(
            "[%(bold)s%(name)-s%(reset)s] %(log_color)s%(levelname)-8s%(reset)s %(blue)s%(message)s",
            datefmt = None,
            reset = True,
            log_colors = {
                "DEBUG": "cyan",
                "INFO": "green",
                "WARNING": "yellow",
                "ERROR": "red",
                "CRITICAL": "red,bg_white",
            },
        )
        if not self.handlers:
            console_handler = logging.StreamHandler()
            console_handler.setFormatter(formatter)
            self.addHandler(console_handler)


def setup_loggers():
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
