#!/usr/bin/env python3
import subprocess
import re

class Performer():
    
    _cmd = ["rosrun", "camera_controls", "controller.py"]
    
    def __init__(self, solution):
        self.commands = self.input_solution(solution)

    def get_commands(self, solution):
        parse_solution = re.sub(' +',' ',re.sub('\t+', '', solution)).split("\n")
        for i in range(len(parse_solution)):
            parse_solution[i] = parse_solution[i].strip()
        return parse_solution

    def input_solution(self, solution):
        self.solution = self.get_commands(solution)

    def perform_solution(self):
        for command in self.solution:
            result = subprocess.run(self._cmd + command.split(" "),timeout=5)
            
            if result.returncode:
                raise Exception("Command '" + command + "' broke it")

if __name__ == '__main__':
    performer = Performer("move drone 0 0 0\nmove_direct     drone 1.34 3.14 346\n move    drone 0120 874 1231\n rotate \t\t drone 1 2 3.5\ntranslate drone 1 2 3.5  4 5.76 6  ")
    performer.perform_solution()
