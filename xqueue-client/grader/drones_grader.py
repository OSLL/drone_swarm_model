import sys
import importlib

sys.path.append("..")
grader = importlib.import_module("xqueue_watcher.grader")

class Grader(grader.Grader):
	def grade(self, grader_path, grader_config, student_response):
		tests = []
		errors = []
		correct = 0
		score = 0
		if grader_path.endswith('/correct'):
			correct = 1
			score = 1
			tests.append(('short', 'long', True, 'expected', 'actual'))
			tests.append(('short', '', True, 'expected', 'actual'))
		elif grader_path.endswith('/incorrect'):
			tests.append(('short', 'long', False, 'expected', 'actual'))
			errors.append('THIS IS AN ERROR')
			errors.append('\x00\xc3\x83\xc3\xb8\x02')

		results = {
			'correct': correct,
			'score': score,
			'tests': [],
			'errors': []
		}
		return results
