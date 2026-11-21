import json
import socket
import threading
import time
import unittest
from dashboard import Feed, stop_feed


class StreamControlsTest(unittest.TestCase):
    def test_stop_closes_connection_and_restart_receives(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            server.settimeout(3)
            for _ in range(2):
                feed = Feed(server.getsockname()[1])
                worker = threading.Thread(target=feed.run)
                worker.start()
                try:
                    conn, _ = server.accept()
                    with conn:
                        conn.settimeout(2)
                        sample = dict(version=1, seq=1, accel_time_s=1, gyro_time_s=1,
                                      accel_m_s2=[0, 0, -9.8], gyro_rad_s=[0, 0, 0])
                        conn.sendall(json.dumps(sample).encode() + b'\n')
                        deadline = time.monotonic() + 2
                        while feed.count == 0 and time.monotonic() < deadline:
                            time.sleep(0.01)
                        self.assertEqual(feed.count, 1)
                        stop_feed(feed)
                        self.assertEqual(conn.recv(1), b'')
                finally:
                    stop_feed(feed)
                    worker.join(timeout=3)
                self.assertFalse(worker.is_alive())


if __name__ == '__main__':
    unittest.main()
