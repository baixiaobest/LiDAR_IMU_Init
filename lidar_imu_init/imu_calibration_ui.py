"""Local, read-only status UI plus a validated gyro-session selection endpoint."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread


class CalibrationHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def start_ui_server(snapshot, on_select, html, port=8765):
    """Serve only on loopback; callbacks must not modify ROS state in HTTP threads."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            pass

        def send_body(self, status, body, content_type):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/':
                self.send_body(200, html, 'text/html; charset=utf-8')
            elif self.path == '/api/status':
                self.send_body(200, json.dumps(snapshot(), allow_nan=False).encode(), 'application/json')
            else:
                self.send_body(404, b'Not found', 'text/plain; charset=utf-8')

        def do_POST(self):
            if self.path != '/api/use-accelerometer':
                self.send_body(404, b'Not found', 'text/plain; charset=utf-8')
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if length < 1 or length > 1024:
                    raise ValueError('Invalid request size')
                request = json.loads(self.rfile.read(length))
                session = request['session']
                if not isinstance(session, str) or len(session) > 100 or not session.startswith('calibration_'):
                    raise ValueError('Choose a listed calibration session')
                on_select(session)  # Enqueue for the ROS executor; no file or state mutation here.
            except (ValueError, KeyError, json.JSONDecodeError) as exc:
                self.send_body(400, json.dumps({'error': str(exc)}).encode(), 'application/json')
                return
            self.send_body(202, b'{"queued":true}', 'application/json')

    try:
        server = CalibrationHTTPServer(('127.0.0.1', port), Handler)
    except OSError:
        if port == 0:
            raise
        server = CalibrationHTTPServer(('127.0.0.1', 0), Handler)
    thread = Thread(target=server.serve_forever, name='imu_calibration_ui', daemon=True)
    thread.start()
    return server, f'http://127.0.0.1:{server.server_port}/'
