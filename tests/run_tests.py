#!/usr/bin/env python

import rostest
from example import TestBareBones

# rostest
rostest.rosrun('maniros', 'test_bare_bones', TestBareBones)
