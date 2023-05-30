import json
import subprocess

from simulator import Simulator
from performer import Performer

def main():
    sim = Simulator()
    sim.start()
    with open("/catkin_ws/solution/solution", "r") as solution:
        code = '\n'.join(solution.readlines())
        performer = Performer(code)
        try:
            performer.perform_solution()
        except Exception as e:
            print(e)

    with open("/catkin_ws/solution/result", "w") as result:
        result.write(json.dumps({
            'correct': 0,
            'score': 0,
            'tests': [],
            'errors': []
        }))
    sim.end()

main()
