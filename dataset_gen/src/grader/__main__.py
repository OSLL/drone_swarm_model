import json

from performer import Performer
from simulator import Simulator


def main():
    sim = Simulator()
    sim.start()
    with open("/catkin_ws/solution/solution", "r", encoding="utf-8") as solution:
        code = "\n".join(solution.readlines())
        performer = Performer(code)
        try:
            performer.perform_solution()
        except RuntimeError as re:
            print(f"perform_solution() error: {re}")

    with open("/catkin_ws/solution/result", "w", encoding="utf-8") as result:
        result.write(json.dumps({"correct": 0, "score": 0, "tests": [], "errors": []}))
    sim.end()


main()
