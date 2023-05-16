import sys
import importlib

import subprocess
import signal
import os
import time
import json
from statsd import statsd

from .primitive_checker import PrimitiveChecker

sys.path.append("..")
grader = importlib.import_module("xqueue_watcher.grader")

## @brief Class for accepting and checking solutions
class Grader(grader.Grader):

    def __init__(self, *args, **kwargs) -> None:
        super(Grader, self).__init__(*args, **kwargs)
        self._proc = None
        self._solution = ""


    ## @brief Grade solution
    # @param[in] grader_path path to wanted grader (unused)
    # @param[in] grader_config JSON object with "grader_payload"
    # @param[in] student_response JSON object with "student_response"
    def grade(self, grader_path, grader_config, student_response):
        #task_id = grader_config["task_id"]
        results = {}
        checker = PrimitiveChecker()
        if checker.check(self._solution):
            self._start_container()
            results = self._exec_simulation_container()
            self._stop_container()
        else:
            results = {
                'score': 0,
                'correct': False,
                'tests': [(
                    'Basic check failed', 
                    'Wrong syntax or empty solution. No drones were launched.', 
                    False, 
                    '-', 
                    '-'
                )],
                'errors': checker.get_errors()
            }
        return results


    ## @brief Processing function, copied from parent class, but uses "xqueue_files" field
    # @param[in] content JSON with received data
    # @param[in] queue current watcher queue
    def process_item(self, content, queue=None):
        try:
            statsd.increment('xqueuewatcher.process-item')
            body = content['xqueue_body']
            files = json.loads(content['xqueue_files'])

            # Delivery from the lms
            body = json.loads(body)
            student_response = body['student_response']
            payload = body['grader_payload']
            try:
                grader_config = json.loads(payload)
            except ValueError as err:
                # If parsing json fails, erroring is fine--something is wrong in the content.
                # However, for debugging, still want to see what the problem is
                statsd.increment('xqueuewatcher.grader_payload_error')

                self.log.debug(f"error parsing: '{payload}' -- {err}")
                raise

            self.log.debug(f"Processing submission, grader payload: {payload}")
            relative_grader_path = grader_config['grader']
            grader_path = (self.grader_root / relative_grader_path).abspath()
            self._solution = files["solution"]              # save solution!
            start = time.time()
            results = self.grade(grader_path, grader_config, student_response)

            statsd.histogram('xqueuewatcher.grading-time', time.time() - start)

            # Make valid JSON message
            reply = {'correct': results['correct'],
                     'score': results['score'],
                     'msg': self.render_results(results)}

            statsd.increment('xqueuewatcher.replies (non-exception)')
        except Exception as e:
            self.log.exception("process_item")
            if queue:
                queue.put(e)
            else:
                raise
        else:
            if queue:
                queue.put(reply)
            return reply


    def _start_container(self, init_time : int = 5):
        if self._proc is None:
            if not os.getcwd().endswith("dataset_gen"):
                os.chdir("../../dataset_gen")
            self._proc = subprocess.Popen(["docker-compose", "up","--detach"])
            if init_time:
                time.sleep(init_time)


    def _exec_simulation_container(self):
        with open("solution/solution", "w") as solution_file:
            solution_file.write(self._solution)
        proc = subprocess.Popen(["docker", "exec", "-it", "dataset_gen", "bash"])

        results = {
            'correct': 0,
            'score': 0,
            'tests': [],
            'errors': []
            }
        start_time = time.time()
        timeout_flag = False
        while('result' not in os.listdir('./solution')):
            time.sleep(3)
            if time.time() - start_time > 60:
                timeout_flag = True
                break
        if timeout_flag:
            return results
        with open("solution/result", "r") as result_file:
            results = json.loads('\n'.join(result_file.readlines()))
            print(results)
        os.remove("solution/result")
        return results


    def _stop_container(self, timeout : int = 20):
        if self._proc is None:
            return
        try:
            self._proc.send_signal(signal.SIGINT)
            self._proc.wait(timeout if timeout > 0 else None)
        except subprocess.TimeoutExpired:
            self._proc.kill()
        finally:
            self._proc = None


if __name__ == '__main__':
    print("This module is not executable")
