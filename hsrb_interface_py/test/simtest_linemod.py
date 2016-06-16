#!/usr/bin/env python
# vim: fileencoding=utf-8 :
"""

"""
import unittest
from tf import transformations
from hsrb_interface import geometry
import testing

class LinemodTest(testing.HsrbInterfaceTest):
    def test_linemod_detector(self):
        """Getting started"""



if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_linemod', LinemodTest)
