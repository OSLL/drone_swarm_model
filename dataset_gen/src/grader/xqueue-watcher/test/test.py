import sys


def callback(response):
	print(response)


if __name__ == "__main__":
	sys.path.append("../")
	import watcher
	import config
	
	watcher = watcher.XQueueWatcher(
		config.CONFIG["CLIENT"],
		callback,
		timeout=0,
		work_time=10000
	)