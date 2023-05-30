#!/usr/bin/env python3
import re
import subprocess
import time


class Performer:
    _cmd = ["rosrun", "camera_controls", "controller.py"]
    _base_time_wait = 3

    def __init__(self, solution):
        self.commands = self.input_solution(solution)

    def get_commands(self, solution):
        parse_solution = re.sub(" +", " ", re.sub("\t+", "", solution)).split("\n")
        for i in range(len(parse_solution)):
            parse_solution[i] = parse_solution[i].strip()
        return parse_solution

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
        )
        if result.returncode:
            raise Exception("Trouble to create launch file")
        proc_sim = subprocess.Popen("roslaunch simulation sim.launch", shell=True)
        time.sleep(self._base_time_wait + int(0.5 * len(drone_names)))
        driver_proc = []
        driver_vel_proc = []
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
    with open("solution/solution", "r") as f:
        solution = f.read()
    performer = Performer(solution)
    # performer = Performer("move drone1 0 0 0\nmove_direct     drone1 1.34 3.14 346\n move    drone1 0120 874 1231\n rotate \t\t drone2 1 2 3.5\ntranslate drone1 1 2 3.5  4 5.76 6  ")
    results = {"correct": 0, "score": 0, "tests": [], "errors": []}
    results = json.dumps(results, indent=4)
    performer.perform_solution()
    with open("solution/result", "w") as f:
        f.write(results)
