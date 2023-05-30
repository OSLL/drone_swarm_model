#! /usr/bin/python3
# -*- coding: utf-8 -*-
## Class for simulation launch

import signal
import subprocess


## @brief Class that launches or stops simulation from given file
class Simulator:
    ## @brief First command-line arguments for the command
    _cmd = ["roslaunch", "simulation"]

    def __init__(self) -> None:
        self._launch_file = "cottage_blender.launch"
        self._proc = None

    ## @brief Start specified simulation in detached process
    # @param[in] launch_file name of simulation launch file
    def start(self, launch_file: str = "") -> None:
        if self._proc:
            raise RuntimeError(
                "simulation " + self._launch_file + " is already started"
            )
        if launch_file != "":
            self._launch_file = launch_file
        self._proc = subprocess.Popen(
            self._cmd + [self._launch_file], stdout=subprocess.DEVNULL
        )

    ## @brief Terminate detached simulation process, if any
    # @param[in] timeout optional time to wait for termination, wait infinitely by default
    def end(self, timeout: int = 0) -> None:
        if not self._proc:
            return
        try:
            self._proc.send_signal(signal.SIGINT)
            self._proc.wait(timeout if timeout > 0 else None)
        except subprocess.TimeoutExpired:
            self._proc.kill()
        finally:
            self._proc = None

    def is_working(self) -> bool:
        return False if not self._proc else self._proc.poll() is None

    def get_launch_file(self) -> str:
        return self._launch_file


if __name__ == "__main__":
    print("This module is not executable")
