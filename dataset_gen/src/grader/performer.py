#!/usr/bin/env python3
import re
import subprocess
import time
import json


class Performer:
    _cmd = ["rosrun", "camera_controls", "controller.py"]
    _base_time_wait = 3

    def __init__(self, solution):
        self.input_solution(solution)
        self.commands = self.solution

    def get_commands(self, solution):
        parse_solution = re.sub(" +", " ", re.sub("\t+", "", solution)).split("\n")
        return [item.strip() for item in parse_solution]

    def input_solution(self, solution):
        self.solution = self.get_commands(solution)

    def get_drone_names(self):
        drone_names = [i.split(" ")[1] for i in self.solution[:-1]]
        return list(set(drone_names))

    def perform_solution(self):
        drone_names = self.get_drone_names()
        result = subprocess.run(
            "rosrun simulation configure_launch.py --headless True --num "
            + str(len(drone_names))
            + " > src/simulation/launch/sim.launch",
            shell=True,
            check=False
        )
        if result.returncode:
            raise RuntimeError("Trouble to create launch file")
        subprocess.Popen("roslaunch simulation sim.launch", shell=True)
        time.sleep(self._base_time_wait + int(0.5 * len(drone_names)))
        driver_proc = []
        for i in drone_names:
            print(i)
            cmd_driver = ["rosrun", "camera_controls", "driver.py", "__name:=" + i]
            d_p = subprocess.Popen(cmd_driver)
            driver_proc.append(d_p)
        proc = subprocess.Popen(self._cmd, stdin=subprocess.PIPE, encoding="utf-8")
        time.sleep(self._base_time_wait)

        solution = self.solution.copy()
        solution.append("exit")
        result = proc.communicate("\n".join(solution))


if __name__ == "__main__":
    print("performer started")
    with open("solution/solution", "r", encoding="utf-8") as f:
        solution = f.read()
    performer = Performer(solution)
    results = {"correct": 0, "score": 0, "tests": [], "errors": []}
    results = json.dumps(results, indent=4)
    performer.perform_solution()
    with open("solution/result", "w", encoding="utf-8") as f:
        f.write(results)
