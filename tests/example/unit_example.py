#!/usr/bin/env python

import unittest
import rostest

## A sample python unit test
class TestBareBones(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self): # only functions with 'test_'-prefix will be run!
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    rostest.rosrun('maniros', 'example_test', TestBareBones)

