#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2

#import cPickle
import pickle
import zlib
import timeit

import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) 

import time
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

def parse_config():
    import json
    with open(BASE_DIR + '/config.json', 'r') as f:
        data = ''.join(f.readlines())
        data = json.loads(data)
    return data

def main():
    camera = cv2.VideoCapture(0)
    #fourcc = cv2.VideoWriter_fourcc(*'X264')
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    fps = 60
    config = parse_config()
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, config['width'])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config['height'])
    camera.set(cv2.CAP_PROP_FPS, fps)
    #camera.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    camera.set(cv2.CAP_PROP_FOURCC, fourcc)

    filename = BASE_DIR + '/' + str(time.time()) + '.avi'
    writer = cv2.VideoWriter(filename, fourcc, fps, (config['width'], config['height']))

    if not camera.isOpened():
        camera.open(0)

    for i in range(fps * 1):
        ret, frame = camera.read()
        writer.write(frame)

    reader = cv2.VideoCapture(filename)

    while camera.isOpened():
        ret, frame = camera.read()
        ret2, frame2 = reader.read()
        
        cv2.imshow('frame', frame)
        cv2.imshow('reader', frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        writer.write(frame)

        height, width = frame.shape[:2]
        print("height: %d, width: %d" % (height, width))
        channel = config['channel']
        rmatrix = cv2.getRotationMatrix2D((width/2, height/2), 180, 1)
        frame = cv2.warpAffine(frame, rmatrix, (width, height))
        print("Affine Transformation :: Rotation 180'")
        frame = frame.flatten()
        print("Transformation :: Flatten")
        # dump bytes
        tic = timeit.default_timer()
        framebytes = pickle.dumps(frame)
        toc = timeit.default_timer()
        print("piclke.dumps(frame): %fs" % (toc - tic))
        # compress bytes
        tic = timeit.default_timer()
        compressed = zlib.compress(framebytes, 5)
        toc = timeit.default_timer()
        print("zlib.compress(framebytes, level=5): %fs" % (toc - tic))
        print("len(framebytes): %d, len(compressed): %d, compressed: %f%%\n---" % (len(framebytes), len(compressed), (len(framebytes) - len(compressed)) / len(framebytes) * 100))

    camera.release()
    reader.release()
    writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()