import threading
import requests
import time
import sys

class XQueueWatcher:
	def __init__(self, client, callback, delay = 5000, work_time = 0, timeout = 0):
		"""
			callback - function that calls every time if has result
			delay (in ms)
			work_time (in ms)
			timeout (in ms)
		"""
		assert len(client) > 0, "Client must be specified"
		assert delay > 0, "Delay must be more than zero"
		assert work_time >= 0, "Working time can't be less than zero"
		assert timeout >= 0, "Response timeout time can't be less than zero"

		self._client = client
		self._callback = callback
		self._delay = delay
		self._timeout = timeout
		self._working = True
		self._init_ttl(work_time)

		self._work()


	def _init_ttl(self, ttl):
		if ttl:
			threading.Timer(ttl / 1000, self._ttl_callback).start()


	def _ttl_callback(self):
		self._working = False


	def _poll(self, client):
		result = None

		try:
			if self._timeout:
				result = requests.post(self._client, timeout=self._timeout)
			else:
				result = requests.post(self._client)

			return {
				"client": client,
				"valid": True,
				"response": result.json()
			}
		except requests.exceptions.Timeout as e:
			print(f"Client request timed out (timeout={self._timeout})", file=sys.stderr)

			return {
				"client": client,
				"valid": False,
				"exception": e
			}
		except requests.exceptions.ConnectionError as e:
			print("Connection to client failed", file=sys.stderr)

			return {
				"client": client,
				"valid": False,
				"exception": e
			}
		except:
			print(f"Unhandled error: {e.what()}", file=sys.stderr)

			return {
				"client": client,
				"valid": False,
				"exception": e
			}


	def _wait_delay(self, start_time):
		current_time = time.time()
		delta_time = start_time + self._delay - current_time
		if delta_time > 0:
			time.sleep(delta_time / 1000)


	def _work(self):
		result = None
		while self._working:
			start_time = time.time()
			result = self._poll(self._client)
			if result["valid"]:
				self._callback(result["response"])
			self._wait_delay(start_time)
		print("XQueue Watcher stopped")
