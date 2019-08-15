#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import unittest

class TestPickleSerialization(unittest.TestCase):

    def test_pickle_vs_cpickle(self):
        import pickle
        import cPickle
        import random
        import timeit
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


if __name__ == "__main__":
    unittest.main()