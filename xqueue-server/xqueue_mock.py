from http.server import HTTPServer, BaseHTTPRequestHandler
import json
    
def get_response(solution=''):
    return {
        'return_code': 0,
        'content': json.dumps({
            'xqueue_header': json.dumps({
                'lms_callback_url': '', 
                'lms_key': 'lms_key', 
                'queue_name': 'drone_queue'
            }),
            'xqueue_body': json.dumps({
                'student_response': '', 
                'grader_payload': json.dumps({
                    'task_id': 'task_id', 
                    'grader': 'grader.py'
                })
            }),
            'xqueue_files': json.dumps({
                'solution': str(solution)
            })
        })
    }

class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.end_headers()
        solution = 'move drone 0 0 0\nmove_direct     drone 1.34 3.14 346\n move    drone 0120 874 1231\n rotate \t\t drone 1 2 3.5\ntranslate drone 1 2 3.5  4 5.76 6  '
        self.wfile.write(json.dumps(get_response(solution)).encode())

    def do_POST(self):
        self.send_response(200)
        self.end_headers()
        self.wfile.write(json.dumps({'return_code': 0, 'content': 'submit'}).encode())


httpd = HTTPServer(('localhost', 8000), SimpleHTTPRequestHandler)
httpd.serve_forever()
