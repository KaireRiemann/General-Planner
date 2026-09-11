#!/usr/bin/env python3
"""Exercise the ROS image -> local mock VLM -> pixel polygon path on an isolated master."""
import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped
from sensor_msgs.msg import CompressedImage


class MockVlm(BaseHTTPRequestHandler):
    def do_GET(self):
        data=json.dumps({'data':[{'id':'test-model'}]}).encode()
        self.send_response(200); self.end_headers(); self.wfile.write(data)

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        assert request['messages'] and request['model'] == 'test-model'
        payload = json.dumps({'choices': [{'message': {'content': json.dumps({
            'windows': [{'points': [[100, 200], [300, 200], [300, 400], [100, 400]]}]
        })}}]}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *_args):
        pass


def main():
    rospy.init_node('vlm_migration_test', anonymous=True)
    server = HTTPServer(('127.0.0.1', 0), MockVlm)
    # Leave the endpoint unavailable first to exercise startup recovery.
    serving=False
    port=server.server_port
    server.server_close()
    ns = '/windowtec_vlm_box_node'
    params = dict(image_topic='/test/vlm/image/compressed', box_topic='/test/vlm/box',
                  model='auto', base_url='http://127.0.0.1:%s/v1' % port,
                  detection_period=.1, max_retries=0, timeout=2.)
    for k, v in params.items():
        rospy.set_param(ns + '/' + k, v)
    received = []
    rospy.Subscriber('/test/vlm/box', PolygonStamped, received.append)
    pub = rospy.Publisher('/test/vlm/image/compressed', CompressedImage, queue_size=1)
    log = open('/tmp/aperture_vlm_smoke.log', 'w')
    proc = subprocess.Popen(['rosrun', 'aperture_detector', 'windowtec_vlm_box_node.py'], stdout=log, stderr=log)
    try:
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        ok, image = cv2.imencode('.jpg', np.zeros((480, 640, 3), dtype=np.uint8))
        assert ok
        msg.data = image.tobytes()
        offline_end=time.monotonic()+2
        while time.monotonic()<offline_end:
            pub.publish(msg); time.sleep(.1)
        assert proc.poll() is None, 'VLM node exited while service was offline'
        server=HTTPServer(('127.0.0.1',port),MockVlm)
        threading.Thread(target=server.serve_forever,daemon=True).start()
        serving=True
        end = time.monotonic() + 10
        while not received and time.monotonic() < end:
            pub.publish(msg)
            time.sleep(.1)
        assert received, 'no VLM polygon; inspect /tmp/aperture_vlm_smoke.log'
        result = received[-1]
        assert result.header.stamp == msg.header.stamp
        assert [(p.x, p.y) for p in result.polygon.points] == [(64., 96.), (192., 96.), (192., 192.), (64., 192.)]
        print('PASS: VLM node loads prompts, consumes compressed image, calls mock endpoint and preserves pixel corners/timestamp')
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        if serving: server.shutdown()
        server.server_close()
        log.close()


if __name__ == '__main__':
    main()
