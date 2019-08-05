#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

import rospy
from flask import Flask, Response, request
from multiprocessing import Process, Pipe
from threading import Thread
from rosvision import ROSVision

app = Flask(__name__)

def generate_frame(conn):
    while True:
        frame = conn.recv()
        yield(
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n'
        )

@app.route('/')
def home():
    return Response('<h1>Welcome to ROS-Flask!</h1>')

@app.route('/stream')
def stream():
    """
    http://0.0.0.0:8080/stream?namespace=1
    """
    parent_conn, child_conn = Pipe()
    namespace = request.args.get("namespace")
    parent_conn.send(namespace)
    rosvision = ROSVision()
    #process = Process(target=rosvision.run, args=(child_conn,))
    #process.start()
    # https://soooprmx.com/archives/9504
    thread = Thread(target=rosvision.run, args=(child_conn,))
    thread.daemon = True
    thread.start()
    return Response(
        generate_frame(parent_conn),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


if __name__ == '__main__':
    rospy.init_node("ros_http_streamer", anonymous=False)
    app.run(host='0.0.0.0', port=3000, threaded=True)#debug=True
