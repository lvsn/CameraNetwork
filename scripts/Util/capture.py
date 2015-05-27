import threading
import subprocess

__author__ = 'jesangfox'
__version__ = "1.0.0"
__maintainer__ = 'jbecirovski'


def Autobracketing():
    """
    Autobracketing (AEB) is a feature of some more advanced cameras, where the camera will take several
    successive shots with slightly different settings. In this case, the autobracketing take two
    successive autobracketing (3 shots per autobracketing) and one more shot.
    """
    def preset():
        """ Preset the ISO, fileformat, etc. """
        subprocess.call('gphoto2 --reset', shell=True)
        # RAW 32; RAW+samall Fine JPG 12
        subprocess.call('gphoto2 --set-config imageformat=32', shell=True)

        # Keep photo in SD card
        subprocess.call('gphoto2 --set-config capturetarget=1', shell=True)
        subprocess.call('gphoto2 --set-config iso=1', shell=True)
        subprocess.call('gphoto2 --set-config autopoweroff="0"', shell=True)

    def refreshCamera():
        """ Refresh camera in every 300 seconds | if conflicted with capture()? """
        print("refreshCamera")
        # threading.Timer(120.0, refreshCamera).start()
        preset()
        print("Done")

    def capturingAEB():
        """ Using auto-exposure bracketing capture images """
        subprocess.call('gphoto2 --set-config /main/capturesettings/aeb="+/- 3" --set-config /main/capturesettings/aperture="16" --set-config /main/capturesettings/shutterspeed="1/1000" --set-config /main/actions/eosremoterelease=2 --wait-event=1s', shell=True)
        subprocess.call('gphoto2 --set-config /main/capturesettings/aperture=4 --set-config /main/capturesettings/shutterspeed="1/30" --set-config /main/actions/eosremoterelease=2 --wait-event=1s', shell=True)
        subprocess.call('gphoto2 --set-config /main/capturesettings/aeb=0 --set-config /main/capturesettings/shutterspeed="1" --capture-image', shell=True)

    def capture():
        """ Take a group of images every 120 seconds """
        print("Capturing")
        threading.Timer(120.0, capture).start()
        capturingAEB()
        print("Done")

    refreshCamera()
    capture()
