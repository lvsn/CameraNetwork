import requests
import time
import os
import pprint


CAM_URL = 'http://192.168.1.1:80'
EXECUTE_URL = '/osc/commands/execute'
STATUS_URL = '/osc/commands/status'
STATE_URL = '/osc/state'


class RicohThetaS:
    # options = 1

    _options = [
        "perture",
        "captureInterval",
        "captureMode",
        "captureNumber",
        "clientVersion",
        "dateTimeZone",
        "exposureCompensation",
        "exposureDelay",
        "exposureProgram",
        "fileFormat",
        "_filter",
        "gpsInfo",
        "_HDMIreso",
        "'iso','isoSupport'",
        "_latestEnabledExposureDelayTime",
        "offDelay",
        "previewFormat",
        "remainingPictures",
        "remainingSpace",
        "remainingVideoSeconds",
        "shutterSpeed",
        "_shutterVolume",
        "sleepDelay",
        "totalSpace",
        "whiteBalance",
        "_wlanChannel"
    ]
    _values_shutterSpeed = {
        # Value Description
        0.00015625: "1/6400", 0.0002: "1/5000", 0.00025: "1/4000", 0.0003125: "1/3200", 0.0004: "1/2500", 0.0005: "1/2000", 0.000625: "1/1600", 0.0008: "1/1250",
        0.001: "1/1000", 0.00125: "1/800", 0.0015625: "1/640", 0.002: "1/500", 0.0025: "1/400", 0.003125: "1/320", 0.004: "1/250", 0.005: "1/200",
        0.00625: "1/160", 0.008: "1/125", 0.01: "1/100", 0.0125: "1/80", 0.01666666: "1/60", 0.02: "1/50", 0.025: "1/40", 0.03333333: "1/30",
        0.04: "1/25", 0.05: "1/20", 0.06666666: "1/15", 0.07692307: "1/13", 0.1: "1/10", 0.125: "1/8", 0.16666666: "1/6", 0.2: "1/5",
        0.25: "1/4", 0.33333333: "1/3", 0.4: "1/2.5", 0.5: "1/2", 0.625: "1/1.6", 0.76923076: "1/1.3", 1: "1", 1.3: "1.3", 1.6: "1.6",
        2: "2", 2.5: "2.5", 3.2: "3.2", 4: "4", 5: "5", 6: "6", 8: "8", 10: "10", 13: "13", 15: "15", 20: "20", 25: "25", 30: "30", 60: "60"}
    _exposurePrograms = {
        # Value Description
        1: "manual",
        2: "automatic",
        4: "shutter priority",
        9: "iso priority"
    }

    _whiteBalance = {
        # Value Description
        "Auto": "auto",
        "Outdoor": "daylight",
        "Shade": "shade",
        "Cloudy": "cloudy-daylight",
        "Incandescent light 1": "incandescent",
        "Incandescent light 2": "_warmWhiteFluorescent",
        "Fluorescent light 1 (daylight)": "_dayLightFluorescent",
        "Fluorescent light 2 (natural white)": "_dayWhiteFluorescent",
        "Fluorescent light 3 (white)": "fluorescent",
        "Fluorescent light 4 (light bulb color)": "_bulbFluorescent"
    }

    def __init__(self, ip=CAM_URL, port=80):
        self.sid = None
        self.ip = ip
        # self.port = port

        self.executeURL = self.ip + EXECUTE_URL
        self.statusURL = self.ip + STATUS_URL
        self.stateURL = self.ip + STATE_URL

        self.startSession()
        self.lastCmdID = ''

    def _request(self, url, data):
        try:
            r = requests.post(url, json=data)
        except Exception, e:
            print(e)
            return None

        if r.status_code == 200:  # OK
            response = r.json()
            return response
        elif r.status_code == 400:  # bad request
            response = r.json()
            print response
            return None
        else:
            print 'response error'
            return None

    def startSession(self):
        data = {'name': 'camera.startSession', 'parameters': {}}
        response = self._request(self.executeURL, data)
        results = response['results']
        sessionId = results['sessionId']
        self.sid = sessionId
        # self.setOption('clientVersion', 2)  ## not support

    def closeSession(self, delay_sec=3):
        data = {'name': 'camera.closeSession', 'parameters': {'sessionId': self.sid}}
        time.sleep(delay_sec)
        response = self._request(self.executeURL, data)
        if response is not None:
            print 'Session closed'

    def getCameraState(self):
        data = {}
        response = self._request(self.stateURL, data)
        state = response['state']
        return state

    def showCameraState(self):
        print self.getCameraState()

    def _getCmdState(self, id):
        # Command execution status
        # Either "done", "inProgress" or "error" is returned
        data = {'id': id}
        response = self._request(self.statusURL, data)
        state = response['state']
        return state

    def getOption(self, option):
        # Acquires the properties and property support specifications
        # option=['iso', 'isoSupport']
        data = {'name': 'camera.getOptions', 'parameters': {'sessionId': self.sid, 'optionNames': [option]}}
        response = self._request(self.executeURL, data)
        print response['results']

    def setOption(self, option, value):
        data = {"name": "camera.setOptions", "parameters": {"sessionId": self.sid, "options": {option: value}}}
        response = self._request(self.executeURL, data)
        if response is None:
            self.showAllOptions()

    def showAllOptions(self):
        print self._options

    def listImages(self):
        # support version 2.0, but cannot setup clientVersion to 2.1 for our thetaS
        data = {"name": "camera.listFiles", "parameters": {"entryCount": 50, "includeThumb": False}}
        response = self._request(self.executeURL, data)
        pprint.pprint(response)

    def takePicture(self):
        # take one image
        # response state is 'inProgress', latestFileUri should wait until the command state is 'done'
        data = {"name": "camera.takePicture", "parameters": {"sessionId": self.sid}}
        response = self._request(self.executeURL, data)
        if response is not None:
            print 'taking picture'
            self.lastCmdID = response['id']
        else:
            print 'fail to take a picture'

    def saveImage(self, fileUri, dstPath=os.path.curdir):
        imageData = self._getImage(fileUri)
        fileName = fileUri.split("/")[1]
        fileName = os.path.join(dstPath, fileName)

        print("Write image to : %s" % fileName)

        with open(fileName, 'wb') as handle:
            for block in imageData.iter_content(1024):
                handle.write(block)
        print "Image saved: " + fileName

    def _getImage(self, fileUri):
        # get Binary data for image
        # fileUri -String  ID of the file to be acquired
        # stream request. won't use the self._request
        data = {"name": "camera.getImage", "parameters": {"fileUri": fileUri}}
        response_binary_data = requests.post(self.executeURL, json=data, stream=True)
        return response_binary_data

    def deleteFile(self, fileUri):
        data = {"name": "camera.delete", "parameters": {"fileUri": fileUri}}
        response = self._request(self.executeURL, data)
        fileName = fileUri.split("/")[1]

        if response is not None:
            print "image deleted: " + fileName

    def latestFileUri(self):
        while self._getCmdState(self.lastCmdID) != 'done':
            time.sleep(0.1)
        state = self.getCameraState()
        latestFileUri = state["_latestFileUri"]
        return latestFileUri

if __name__ == '__main__':
    thetasTest = RicohThetaS(CAM_URL)
    thetasTest.getCameraState()
    thetasTest.setOption('_shutterVolume', 1)
    thetasTest.setOption('captureMode', 'image')
    thetasTest.setOption('exposureProgram', 1)
    thetasTest.setOption('exposureCompensation', 0.0)
    thetasTest.getOption('fileFormat')
    thetasTest.setOption('iso', 100)
    thetasTest.setOption('shutterSpeed', 0.05)

    thetasTest.takePicture()
    thetasTest.saveImage(thetasTest.latestFileUri())
    thetasTest.deleteFile(thetasTest.latestFileUri())

    thetasTest.closeSession()

# $ export FLASK_APP=hello.py
# $ flask run
#  * Running on http://127.0.0.1:5000/

# app = Flask(__name__)


# @app.route('/')
# def index():
#     return 'Index Page'


# @app.route('/hello')
# def hello():
#     return 'Hello, World'


# @app.route('/thetas')
# def thetaS():
#     return 'hello thetaS  '


# def startSession():
