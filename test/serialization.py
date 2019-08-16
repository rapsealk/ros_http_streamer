#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import unittest

import cv2

import cPickle
import pickle
import random
import timeit

class TestPickleSerialization(unittest.TestCase):

    def test_pickle_vs_cpickle(self):
        data = bytearray([random.randint(0, 127) for i in range(1024 * 1024)])
        
        tic = timeit.default_timer()
        pickle.dumps(data)
        toc = timeit.default_timer()
        pickle_time = toc - tic
        print('pickle.dumps:', pickle_time)
        
        tic = timeit.default_timer()
        cPickle.dumps(data)
        toc = timeit.default_timer()
        cpickle_time = toc - tic
        print('cPickle.dumps:', cpickle_time)

        self.assertLess(cpickle_time, pickle_time)

    def test_cv_dumping(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            cap.open(0)
        i = 0
        pickle_time = 0
        cpickle_time = 0
        while cap.isOpened() and i < 100:
            i += 1
            ret, frame = cap.read()
            print('frame.shape:', frame.shape)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            frame = frame.flatten()
            tic = timeit.default_timer()
            pickle.dumps(frame)
            toc = timeit.default_timer()
            time = toc - tic
            print('pickle.dumps(frame):', time)
            pickle_time += time
            tic = timeit.default_timer()
            cPickle.dumps(frame)
            toc = timeit.default_timer()
            time = toc - tic
            print('cpickle.dumps(frame):', time)
            cpickle_time += time
            print('---')

        print('pickle:', pickle_time)
        print('cpickle:', cpickle_time)
        self.assertLess(cpickle_time, pickle_time)

if __name__ == "__main__":
    unittest.main()
