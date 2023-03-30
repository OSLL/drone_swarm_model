#!/usr/bin/env python3
import os
import re

class Performer():
    def __init__(self, solution):
        self.commands = self.input_solution(solution)

    def get_commands(self, solution):
        temp = re.sub(' +',' ',re.sub('\t+', '', solution)).split("\n")
        for i in range(len(temp)):
            temp[i] = temp[i].strip()
        return temp

    def input_solution(self, solution):
        self.solution = self.get_commands(solution)
    
    def perform_solution(self):
        try:
            for command in self.solution:
                os.system('rosrun camera_controls controller.py ' + command)
        except Excaption as e:
            print(e)

if __name__ == '__main__':
    performer = Performer("move drone 0 0 0\nmove_direct     drone 1.34 3.14 346\n move    drone 0120 874 1231\n rotate \t\t drone 1 2 3.5\ntranslate drone 1 2 3.5  4 5.76 6  ")
    performer.perform_solution()
