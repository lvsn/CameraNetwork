#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed May 14 14:58:49 2014

@author: mathieugaron
"""

import subprocess

# if the executable is installed properly it will work
gphoto2Executable = 'gphoto2'

def run(cmd) :
    
    cmd = gphoto2Executable + cmd

    p = subprocess.Popen(cmd, shell=True,executable="/bin/bash",
                         stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE,
                       )
      
    (stdout, stderr) = p.communicate()
    ret = p.returncode
      
    if ret == 1:
        if 'No camera found' in stderr:
            raise RuntimeError('Error talking to the camera: ' + stderr)
      
        return stdout
  
    return stdout
