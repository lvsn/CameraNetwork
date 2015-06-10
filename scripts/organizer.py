#!/usr/bin/env python

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from scripts.Util.organize import *

__author__ = 'jbecirovski'

if __name__ == '__main__':
    path_src = sys.argv[1] + '/'
    organize_folder(path_src)
