#!/usr/bin/env python

import rostest
from example import TestBareBones
from control_adapter_test import ControlAdapterTest

# rostest
rostest.rosrun('maniros', 'test_bare_bones', TestBareBones)
rostest.rosrun('maniros', 'test_basic_node_actions', ControlAdapterTest)

