#!/usr/bin/python
import os, sys, time

if os.geteuid() != 0:
    os.execvp("sudo", ["sudo"] + sys.argv)
try:
    import RPi.GPIO as GPIO
except ImportError:
    pass

if __name__ == '__main__':
    if 'RPi.GPIO' in sys.modules:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(5, GPIO.OUT)
        GPIO.output(5, False)
        time.sleep(5)
        GPIO.output(5, True)
        time.sleep(5)
        GPIO.cleanup()
    else:
        sys.stdout.write('GPIO 5 False (simul)\n')
        time.sleep(1)
        sys.stdout.write('GPIO 5 True (simul)\n')
        time.sleep(5)
        sys.stdout.write('reset done')
