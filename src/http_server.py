#!/usr/bin/python3
# -*- coding: utf-8 -*-
from flask import Flask, Response, request
from multiprocessing import Process, Pipe
from roscv import ROSVision

app = Flask(__name__)

def generate_frame(conn):
    while True:
        frame = conn.recv()
        yield(
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n'
        )

@app.route('/stream')
def stream():
    """
    http://0.0.0.0:8080/stream?namespace=1
    """
    parent_conn, child_conn = Pipe()
    namespace = int(request.args.get("namespace"))
    parent_conn.send(namespace)
    rosvision = ROSVision()
    process = Process(target=rosvision.run, args=(child_conn,))
    process.start()
    return Response(
        generate_frame(parent_conn),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)