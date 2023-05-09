import json
import subprocess

from simulator import Simulator

def main():
    sim = Simulator()
    sim.start()
    with open("/catkin_ws/solution/solution", "r") as solution:
        code = '\n'.join(solution.readlines())
        # TODO: pass code to Performer
    with open("/catkin_ws/solution/result", "w") as result:
        result.write(json.dumps({
            'correct': 0,
            'score': 0,
            'tests': [],
            'errors': []
        }))
    sim.end()

main()