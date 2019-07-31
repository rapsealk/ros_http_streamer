#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
from abc import ABC, abstractmethod

if sys.platform == "linux":
    import picamera
    import picamera.array
else:
    import cv2

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720

class BaseCamera(ABC):

    @abstractmethod
    def capture(self):
        pass

class RpiCamera(BaseCamera):
    
    def __init__(self, size=(IMAGE_WIDTH, IMAGE_HEIGHT)):
        self.camera = picamera.PiCamera()
        self.camera.resolution = size
        self.stream = picamera.array.PiRGBArray(self.camera)

    def capture(self):
        while True:
            self.camera.capture(self.stream, 'bgr', use_video_port=True)
            shape = self.stream.array.shape # (height, width, channel)
            yield self.stream.array
            self.stream.seek(0)
            self.stream.truncate()

class CvCamera(BaseCamera):

    def __init__(self, size=(IMAGE_WIDTH, IMAGE_HEIGHT)):
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        #self.camera.set(cv2.CAP_PROP_FPS, FPS)

    def __del__(self):
        self.camera.release()
        cv2.destroyAllWindows()

    def capture(self):
        from PIL import Image
        while self.camera.isOpened:
            ret, frame = self.camera.read()
            print(frame, frame.shape)
            image = Image.fromarray(frame)
            print("image:", image)
            """ REQUIRED! =======================
            cv2.imshow("CvCamera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            ================================= """
            yield frame

def get_camera():
    global IMAGE_WIDTH, IMAGE_HEIGHT
    if sys.platform == "linux":
        return RpiCamera(size=(IMAGE_WIDTH, IMAGE_HEIGHT))
    else:
        return CvCamera(size=(IMAGE_WIDTH, IMAGE_HEIGHT))


if __name__ == "__main__":
    camera = get_camera()    
    generator = camera.capture()
    for i in range(100):
        frame = next(generator)