from unittest import TestCase
import sys, os

# Active external execution
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
from scripts.Util.capture import capture_AEB_with_preset
capture_AEB_with_preset()

__author__ = 'jbecirovski'


class TestCapture_AEB_with_preset(TestCase):

    def test_capture_AEB_with_preset(self):
        self.fail()
