print("test main+.py june 28 2026")
# location of ei model inside rpi
# /home/machine2/.ei-linux-runner/models/746841/v4-quantized-runner-linux-aarch64

# THIS VERSION:
# CLASSIFICATION AFTER TRAINING AT 90 FLIP AND ALSO FLIPPING EACH IMAGE BEFORE CLASSIFY 90 CLOCKWISE
# CLASSIFICATION with darker images helped and caps are identifiable now. HOWEVER, NOT USEFUL.
# TRY ANOMALY DETECTION, THEN CLASSIFICATION: WHAT TYPE, EMPTY, MISALIGNED OR CAP

import os  #temporary for testing?
from pathlib import Path
import json


LOG_FILE = Path("/tmp/m2.log")
with open(LOG_FILE, "w") as f:
    f.write("=== M2 Boot ===\n")

import time
from datetime import datetime  # add at the top of your file
start_time = time.perf_counter()

def log(msg):
    line = f"{time.strftime('%H:%M:%S')}  {str(msg)}"
    print(line)
    with LOG_FILE.open('a') as f:
        f.write(line + "\n")

def stamp(msg):
    print(f"{msg}: {time.perf_counter() - start_time:.3f}s")
stamp('start')




# import socket
import serial
import threading
import select
import sys
print("importing time, math")
# import time
import math
# print("importing np")



import numpy as np
stamp('import numpy')

from edge_impulse_linux.image import ImageImpulseRunner

# print("importing cv2")
import cv2
stamp('import cv2')

# file paths
# capRecognModel = "/home/machine2/modelfile.eim" #
capRecognModel = "/home/machine2/ei-linux-runner./models/746841/v4-quantized-runner-linux-aarch64"
CMD_FILE = Path("/tmp/m2_command.json")
STATUS_FILE = Path("/tmp/m2_status.txt")

#flags
test = False
angle = 0 #4.2
lower = [50, 70, 70]
u = [100, 255, 255]
g = 7
e = 66600
r = []
share = False
shareData = ""
tcp = False
cam = object
deltaX = 0
deltaY = 0
maxPixelErrorX = 80  # 40 pixels for x axis
maxPixelErrorY = 60  # 30 pixels for y axis
# add - algo to only select the botton edge; compare edge to the centroid and pick the one with lower y
# add - take test shots after micro adjustment; label
# add - special code for 12 port
# add - command to reset rpi from arduino

roiYStart = 190 #110 + 65
roiYEnd = 385 #320 + 65
roiXStart = 55 # define ROI, 50. cropping of orignal in pixels
roiXEnd = 205 # define roi, x end, making a roi 135pix wide #185

verticalLineTarget = 135  # auto mode targets vertical line was 102
horizontalLineTarget = 185
centroidTarget = 72
centroidTargetY = 120
pvAdj = 20   # needed?
phAdj = 30

# control variables
autoAdjust = True  # toggle automatic microadjustments or not.
verbose = True

# ML objects
runner = ImageImpulseRunner("/home/machine2/capModel5.eim")

def reconnect_serial():
    # re-establishes serial manually
    global ser

    try:
        ser.close()
    except Exception:
        pass

    time.sleep(1)

    ser = serial.Serial("/dev/serial0", 9600, timeout=1)
    time.sleep(2)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ser.write(b"PING\n")
    ser.flush()

    log("Serial reopened; PING sent")

def calibrateIllumination():
    illuminationDelta = 0
    img = run(cam, "port", 0, 1)  # True is for preview, return a cropped image as 'camera sees it'

    if img is not None:
        referenceImage = cv2.imread('img/calibration/referenceIllumination.jpg')
        if referenceImage is not None:
            imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            referenceImageGray = cv2.cvtColor(referenceImage, cv2.COLOR_BGR2GRAY)
            differenceImage = cv2.absdiff(imgGray, referenceImageGray)
            imgBrightness = np.mean(imgGray)
            referenceImageBrightness = np.mean(referenceImageGray)
            illuminationDelta = int(imgBrightness - referenceImageBrightness)
            deltaIllum = round(referenceImageBrightness / imgBrightness, 2)
            # cv2.imshow("illumination sample", img)

            # cv2.imshow("difference to ref", differenceImage)
            # cv2.waitKey(1)
            gainString = str(deltaIllum) + '\n'
            ser.write(gainString.encode())
            print(f"illumination adj: {deltaIllum}")
            log(f"illumination adj: {deltaIllum}")

            return

        else:
            print('refrence image did not load')
            return


def classify(img):
    global runner
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    if img is None:
        return 'no image'
    runner.init()
    features, cropped = runner.get_features_from_image(img)
    result = runner.classify(features)
    decision = 'cap' if result['result']['classification']['cap'] > result['result']['classification']['port'] else 'port'
    return decision

def run(cam, portName="port", portNumber = 2, preview = 0):
    global angle, lower, u, g, prev, e, share, shareData, deltaX, deltaY, verticalLineTarget, horizontalLineTarget,pvAdj,phAdj
    cxLeftmost = 0  # x position of centroid
    portColumn = portNumber % 12
    CX = 60
    CY = 145
    deltaCX = 0
    cyMean = 145
    rightmostLine = [0, 0, 70]

    # stamp('before camstart in run')
    # cam.start()
    # stamp('after cam start')
    try:
        if not test:
            frame = cam.capture_array()
            frame = frame[roiYStart:roiYEnd, roiXStart:roiXEnd]  #frame[105:332, :152]
        else:  #if testing
            if len(imgFiles) > 0:
                if share:
                    imgPath = "/Users/saidyakhyoev/rpi open cv folder/snapshots/forTest.jpg"  # + imgFiles.pop()
                    print(imgPath)
                    share = False
            frame = cv2.imread(imgPath)  # 81,164

        # Rotate to make the image level
        (h, w) = frame.shape[:2]
        center = (w // 2, h // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_image = cv2.warpAffine(frame, rotation_matrix, (w, h))

        # extract gray for brightness check
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = gray[roiYStart:roiYEnd, roiXStart:roiXEnd] #gray[10:300, :135]
        sampleBrightness = np.mean(gray)

        # Convert to HSV
        hsv = cv2.cvtColor(rotated_image, cv2.COLOR_BGR2HSV)

        # Apply Gaussian blur
        hsv_blurred = cv2.GaussianBlur(hsv, (3, 3), 0)

        # Define green range in HSV (adjust these values!)
        lower_green = np.array(lower)  # [50, 70, 70] Adjust these values!
        upper_green = np.array(u)  # [85, 255, 255] Adjust these values!

        # Create mask
        mask = cv2.inRange(hsv_blurred, lower_green, upper_green)
        # Morphological operations
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        overlay = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

# crop image?
#         mask = mask[10:300, roiXStart:roiXEnd]  # picamera [y,x]   mask[10:300, :135]
#         overlay = overlay[10:300, roiXStart:roiXEnd] # overlay[10:300, :135]
        if preview == 1:
            return frame

        if preview == 2:
            return overlay


        # Find contours and centroid
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # largest_contour = max(contours, key=cv2.contourArea)
            # sorted_contours = sorted(contours, key=cv2.contourArea)
            # largest_contour = sorted_contours[-1]
            # filter out small noise contours.
            centroidAggregatorY = []
            centroidAggregatorX = []
            for ct in contours:
                if cv2.contourArea(ct) > 1000:  # adjust 100 to the minimum desired area.
                    x, y, w, h = cv2.boundingRect(ct)
                    cx = int(x + w / 2)
                    cy = int(y + h / 2)
                    centroidAggregatorX.append((x, w))
                    centroidAggregatorY.append((y, h))
                    cv2.drawMarker(overlay, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_TRIANGLE_DOWN, markerSize=7)

            # centroid y
            if len(centroidAggregatorY) > 1:
                CY = int(min(centroidAggregatorY)[0] + max(centroidAggregatorY)[1]/2)  #min y and max width. used by cyMean
                # cyMean = int(sum(centroidAggregatorY) / len(centroidAggregatorY))
            if len(centroidAggregatorY) == 1:
                CY = int(centroidAggregatorY[0][0] + (centroidAggregatorY[0][1]/2))  #y center of the only contour/blob/counding box

            # centroid x
            if len(centroidAggregatorX) == 1:
                CX = centroidAggregatorX[0][0] + int(centroidAggregatorX[0][1]/2)
            elif len(centroidAggregatorX) > 1:
                if portColumn == 0:
                    rightXW = sorted(centroidAggregatorX)[-1]  #rightmost contour X and width tupple: (x,w)
                    cxRightmost = int(rightXW[0] + 52)  #
                    CX = cxRightmost
                else:
                    averageXLeftmost = int((sorted(centroidAggregatorX)[0][0] + sorted(centroidAggregatorX)[1][0]) / 2) # avg of x of left contour (selects left blob)
                    averageWidthLeftmost = int((sorted(centroidAggregatorX)[0][1] + sorted(centroidAggregatorX)[1][1]) / 2)
                    cxLeftmost = averageXLeftmost - (52 - averageWidthLeftmost)  #52 is the connector width /2
                    CX = cxLeftmost
            else:
                if not classify(frame):
                    print('cap or obstruction')
                    log('cap or obstruction')

                    ser.write("100\n100\n".encode())
                    cv2.putText(overlay, "CX: ?", (CX + 10, cyMean), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1)

            # draw CX if visible
            if 0 < CX < 130:
                if CY is None:
                    CY = cyMean
                cv2.drawMarker(overlay, (CX, CY), (0, 255, 0), markerType=cv2.MARKER_SQUARE, markerSize=10)

            # draw centroid target
            cv2.drawMarker(overlay, (centroidTarget, 120), (0, 255, 255), markerType=cv2.MARKER_DIAMOND, markerSize=10)

# rough alignment check. If connector is out of scope of vision partially, do rough adjustment first
            deltaCX = centroidTarget - CX  #centroid of rightmost contour (port 12, 24 etc)
            deltaCY = centroidTargetY - CY

            # TEST
            cv2.putText(overlay, f"dCX:{deltaCX}", (42, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(overlay, f" TG,CX:{centroidTarget, CX}", (5, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        else:  # if no contours identified, possibly anomaly (cap, misplaced port, obscured or illumination etc)
            res = classify(frame)
            print(res)
            if res == 'cap':
                ser.write("100\n100\n".encode())  # special case, return cap detected. stop.
                return

# fine adjust, when image is already aligned
        #Detect edges in the cropped mask
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)

        # scale line
        cv2.line(overlay, (verticalLineTarget, 0), (verticalLineTarget, 225), (255, 255, 255), 1, cv2.LINE_AA)  # vertical target line
        cv2.line(overlay, (0, horizontalLineTarget), (175, horizontalLineTarget), (255, 255, 255), 1, cv2.LINE_AA)  # horizontal target line
        cv2.putText(overlay, f"{verticalLineTarget}", (verticalLineTarget - 25, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (255, 255, 255), 1)
        cv2.putText(overlay, f"{horizontalLineTarget}", (15, horizontalLineTarget + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (255, 255, 255), 1)
        ## > mark lines
        linesP = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 50)
        if linesP is not None:
            verticalLines = []
            horizontalLines = []
            # Rightmost line
            for i in range(0, len(linesP)): # sort out onl long lines
                dX = linesP[i][0][2] - linesP[i][0][0] # delta x
                dY = linesP[i][0][3] - linesP[i][0][1] # delta y
                if dX < 10:
                    verticalLines.append(linesP[i][0])
                elif dY < 10:
                    horizontalLines.append(linesP[i][0])
            if len(verticalLines) > 0:
                verticalLines = sorted(verticalLines, key=lambda x: x[2])
                rightmostLine = verticalLines[-1]  # line with the largest x coordinate, i.e. righmost line
                cv2.line(overlay, (rightmostLine[0], rightmostLine[1]), (rightmostLine[2], rightmostLine[3]), (0, 255, 0), 1, cv2.LINE_AA)
                # cv2.putText(overlay, f"R:{rightmostLine[2]}", (5, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255), 1)
                cv2.putText(overlay, f"dx :{verticalLineTarget - rightmostLine[2]}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0, 0, 255), 1)
                deltaX = rightmostLine[2]-verticalLineTarget  # x target

                # if portColumn == 0:
                #     deltaX += 40  # shift camera to the right, est. widge of connector

                # ser.write(deltaX.encode())
            else:
                deltaX = rightmostLine[2] - verticalLineTarget


            # Bottom line
            if len(horizontalLines) > 0:
                # eliminate lines that are above the centroid, since we need the bottom edge of the connector
                horizontalLines = [y for y in horizontalLines if y[3] > cyMean]
                if len(horizontalLines) > 0:
                    horizontalLines = sorted(horizontalLines, key=lambda y: y[3])
                    # print("horizontal lines, length:", horizontalLines)
                    bottomLine = horizontalLines[-1]
                else:
                    bottomLine = [0,175,52,175]  # if no lower edge found, substitute artificial line

                # print("bottomLine", bottomLine)
                # print("cyMean", cyMean)
                cv2.line(overlay, (bottomLine[0], bottomLine[1]), (bottomLine[2], bottomLine[3]), (255, 0, 0), 2, cv2.LINE_AA)
                # cv2.putText(overlay, f"B:{bottomLine[3]}", (5, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                cv2.putText(overlay, f"dy:{horizontalLineTarget - bottomLine[3]}", (5, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                if bottomLine[3]:
                    deltaY = bottomLine[3]-horizontalLineTarget
                else:
                    deltaY = 0

            # Display the frame (optional)

        if "cor" in portName:
            cv2.imshow("after 1st correction", overlay)
        elif "conf" in portName:
            cv2.imshow("final position. confirm only", overlay)
            fn = "img/good/" + str(deltaX) + '_' + str(deltaY) + '_final' + str(portNumber) + ".jpg"
        else:
            cv2.imshow("init overlay", overlay)
            # fn = "img/raw/port.port" + str(portNumber) + ".jpg"
            # # save image
            # if abs(int(deltaX)) > maxPixelErrorX or abs(int(deltaY)) > maxPixelErrorY:  #if error is large, check with classifier
            #     label = classify(frame)  # if pixel error is too large, use clasifier to underderstand recognize.
            #     if label == 'cap':
            #         print(label)
            #         return
            #     # fn = f"img/raw/{label}.port" + str(portNumber) + ".jpg"
            # fn = f"img/raw/port{portNumber}_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
            # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            # cv2.imwrite(fn, frame)
            bad_aim = (
                    abs(int(deltaX)) > maxPixelErrorX
                    or abs(int(deltaY)) > maxPixelErrorY
            )

            folder = "img/bad" if bad_aim else "img/good"
            fn = f"{folder}/port{portNumber}_{deltaX},{deltaY}_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
            # saved_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            cv2.imwrite(fn, frame)

            if bad_aim:
                label = classify(frame)
                if label == 'cap':
                    print(label)
                    return

        cv2.waitKey(1)

# rough adjust if off center
        if abs(deltaCX) > 23:  # if left connector centroid 10 px left or right, reposition probe
            deltaCXcorrection = str(-deltaCX) + "\n"
            if autoAdjust:
                ser.write(deltaCXcorrection.encode())  # rough adj x coord to arduino
                ser.write("0\n".encode())  # rough adj y to arduino
            if verbose:
                print('rough adjust. deltaX', deltaX, 'deltaY', deltaY)
        # if abs(deltaCY) > 13:
        #     deltaCYcorrection = str(-deltaCY) + '\n'
        #     print(f"rough adj y: {deltaCYcorrection.strip(), str(0)}")
        #     if autoAdjust:
        #         ser.write(deltaCXcorrection.encode())  # rough adj x coord to arduino
        #         ser.write("0\n".encode())  # rough adj y to arduino
        else:
# standard adj. send probe adjustments to arduino
            deltaX = str(deltaX) + "\n"
            deltaY = str(deltaY) + "\n"

            if autoAdjust:
                if not "conf-" in portName:
                    ser.write(deltaX.encode())  # delta x px from expected edge, str
                    ser.write(deltaY.encode())  # delta y, str

                    if verbose: print(f"standard adjust:{deltaX.strip(), deltaY.strip()}")

    except KeyboardInterrupt:
        print("run function interrupted")

    finally:
        pass




# MAIN CODE - runs first
print("main code")
log("main code ")

# modes
calibrationMode = False


if test:  # load images from folder
    imgFiles = os.listdir("/Users/saidyakhyoev/rpi open cv folder/snapshots/")
    imgPath = "/Users/saidyakhyoev/rpi open cv folder/snapshots/snapshot.jpg"
else:
    try:
        print("importing picamera2")
        log("importing picamera2 ")
        from picamera2 import Picamera2

        # Serial comm
        ser = serial.Serial('/dev/serial0', 9600, timeout=1) #open serial port
        # ser.write(b'RPi ready\n')  # send string to Arduino



        # setup camera
        cam = Picamera2()
        configRGB = cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        cam.configure(configRGB)
        cam.set_controls({'ExposureTime': e, 'AnalogueGain': g, 'AeEnable': True})
        #ser.write('home\n'.encode())
        line = '{"commandType":10}'.encode()
        ser.write(line)
        print("ready v4-double adj. VTL = 150")
        log("ready...")
        typed = ""

        cam.start()  #
        time.sleep(0.1)  #



        while True:
            img = cam.capture_array()  #
            # img = img[105: 330, :150]  #pre-crop
            # img = img[10:300, roiXStart:roiXEnd]  # crop further
            # wholeimg = img
            img = img[roiYStart:roiYEnd, roiXStart:roiXEnd]  # crop further

            cv2.line(img, (verticalLineTarget, 0), (verticalLineTarget, 225), (255, 255, 255), 1, cv2.LINE_AA)  # vertical target line
            cv2.line(img, (0, horizontalLineTarget), (175, horizontalLineTarget), (255, 255, 255), 1, cv2.LINE_AA)  # horizontal target line

            cv2.line(img, (verticalLineTarget-pvAdj, 0), (verticalLineTarget-pvAdj, 225), (0, 255, 255), 1, cv2.LINE_AA)  # vertical target line
            cv2.line(img, (0, horizontalLineTarget-phAdj), (175, horizontalLineTarget-phAdj), (0, 255, 255), 1, cv2.LINE_AA)  # horizontal target line

            if calibrationMode:
                img = run(cam, "port ", 0, 2)

            cv2.imshow("preview", img)
            cv2.waitKey(1)

            if ser.in_waiting >= 2:
                msg = ser.readline().decode('utf-8').rstrip()

                if "CHK" in msg:
                    portName = msg[3:]
                    portNumber = int(portName)
                    print()
                    print(f"port chk: {portName}")
                    run(cam, portName, portNumber)  # take pic and find port misalignment in pixels
                    # ser.write("lights\n".encode())

                elif "COR" in msg:
                    portName = msg[3:]
                    portNumber = int(portName)
                    print(f"port cor: {portName}")
                    run(cam, "cor-"+portName, portNumber)

                elif "CONFRM" in msg:
                    portName = msg[6:]
                    portNumber = int(portName)
                    print("final position. for confirm only")
                    run(cam, "conf-" + portName, portNumber)

                elif "rpi:" in msg:
                    msg = msg[4:]
                    print(f"ard>rpi: {msg}")
                    # log(f"ard>rpi: {msg}")
                    os.system(msg)

                elif msg == "ci":  # calibrate illumination
                    print(f"ardu: {msg}")
                    # log(f"ardu: {msg}")
                    calibrateIllumination()
                    calibrationMode = True


                elif len(msg) > 1:
                    print("ardu: ", msg)

            # read Flask /Tailscale server message
            if CMD_FILE.exists():
                data = json.loads(CMD_FILE.read_text())
                CMD_FILE.unlink()

                line = data.get("cmd", "").strip()
                testMode = data.get("mode", "detect").strip()

                print("web command:", line, testMode)
                # log("web command:", line, testMode)
                STATUS_FILE.write_text(f"main+.py received command: {line}, mode: {testMode}")

            # read user input
            if select.select([sys.stdin], [], [], 0.0)[0]:  # a tuble of 3 lists: readable, writable, error. use first, readabe
                line = sys.stdin.readline().strip()

                # adjust line
                if line == "restart":
                    print("got restart")
                    log("got restart arduino")
                    line = '{"commandType":10}'.encode()
                    ser.write(line)
                elif line.endswith('.'):
                    command = '{"commandType":2,"port":' + line.rstrip('.') + '}'
                    command = command.encode()
                    ser.write(command)
                # elif testMode == "detect":
                #     command = '{"commandType":2,"port":' + line.rstrip('.') +  '}'
                #     log(command)
                #     command = command.encode()
                #     ser.write(command)

                elif line.isdigit():
                    if testMode == "detect":
                        command = '{"commandType":2,"port":' + line.rstrip('.') + '}'
                    elif testMode == "lase":
                        command = '{"commandType":1,"port":' + str(line)+'}'
                    log('/\n' + command)
                    command = command.encode()
                    ser.write(command)
                elif line == "reconnect":
                    reconnect_serial()
                    log("reconnect serial")

                elif line == 'r':
                    verticalLineTarget -= 1
                    print('verticalLineTarget:',str(verticalLineTarget), 'probe vert adj:',str(pvAdj))
                    # log('verticalLineTarget:',str(verticalLineTarget), 'probe vert adj:',str(pvAdj))
                    log("vertical line -1")
                elif line == 'l':
                    verticalLineTarget += 1
                    print('verticalLineTarget:', str(verticalLineTarget),'probe vert adj:',str(pvAdj))
                    # log('verticalLineTarget:', str(verticalLineTarget),'probe vert adj:',str(pvAdj))
                    log("vertical line+1")

                elif line == 'u':
                    horizontalLineTarget -= 1
                    print('horizontalLineTarget:', str(horizontalLineTarget),'probe hor adj:',str(phAdj))
                elif line == 'd':
                    horizontalLineTarget += 1
                    print('horizontal:', str(horizontalLineTarget),'probe hor adj:',str(phAdj))

                elif line == '.':
                    pvAdj -= 1
                    print('probe vert adj:',str(pvAdj))
                elif line == ',':
                    pvAdj += 1
                    print('probe vert adj:',str(pvAdj))
                elif line == "autoAdjust":
                    autoAdjust = True
                    print("autoAdjust")
                elif line == "autoAdjustOff":
                    autoAdjust = False
                    print("autoAdjust off")
                elif line == "image":
                    img = cam.capture_array()
                    cv2.imwrite('img/calibration/temp_image.jpg', img)
                    print('snapshot saved img/calibration/temp_image.jpg')
                elif line == "ci":  # for calibrate illumination
                    # print(f"delta from calibrated: {calibrateIllumination()}")
                    ser.write("ci".encode())
                elif line == "snapshot":
                    img = run(cam, "port", 0, 1)  # run camera, name image 'port', init port = 0, preview 1st
                    cv2.imwrite('img/calibration/referenceIllumination.jpg', img)
                    print('snapshot saved img/calibration/referenceIllumination.jpg')

                else:
                # else pass to the serial port as bytes
                    line = line.encode()

                # decodedLine = line.split("port")
                # if decodedLine[0] == '':
                #     decodedLine[0] = '1'
                #
                # command = ': : {"commandType":' + decodedLine[0] + ', "port":' + decodedLine[-1] + '}'
                # print('command sent:', command)
                # command = command.encode()
                # ser.write(command)  # : : is needed because of how ardu expect data from esp
                    ser.write(line)



    except KeyboardInterrupt:
        cam.stop()
        ser.close()
        runner.stop()
        print("Serial port closed.")
        quit()

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# send_queue = queue.Queue()
# recv_queue = queue.Queue()


#
# def tcp_client():
#     global shareData, tcp, share
#     # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     # connect
#     if not tcp:
#         print("connecting to server..")
#         try:
#             s.connect(("34.57.37.242", 4000))
#             s.setblocking(False)
#             s.sendall(b"Mac\n\r")
#             print("connected")
#             tcp = True
#             # while True:
#             #     time.sleep(0.1)
#             #     ready, _, _ = select.select([s], [], [], 0)
#             #     if ready:
#             #         try:
#             #             data = s.recv(1024)
#             #             if data:
#             #                 # recv_queue.put(data.decode())
#             #                 print(data.decode())
#             #         except:
#             #             pass
#             #     # if shareData != "":
#             #     try:
#             #         msg = send_queue.get_nowait()
#             #         print("shareData",shareData)
#             #         try:
#             #             s.sendall(str(shareData).encode() + b"\n")
#             #             print("sent:", shareData)
#             #         except (BrokenPipeError, ConnectionResetError) as e:
#             #             print("Socket closed unexpectedly:", e)
#             #             break  # exit the loop
#             #         time.sleep(0.1)
#             #
#             #         # shareData = ""
#             #
#             #     except queue.Empty:
#             #         pass
#         except Exception as e:
#             print("TCP error:", e)
#
#     else:  # if tcp connection is established
#         try:
#             data = s.recv(1024)
#             if data:
#                 # recv_queue.put(data.decode())
#                 print(data)
#         except:
#             pass
#         if share:  # send only when i toggle share True manually
#             try:
#                 print("sending")
#                 s.sendall(b"hey there")
#
#                 share = False
#             except Exception as e:
#                 print(e)

# def click_event(event, x,y,flags,param):
#     global r
#     if event == cv2.EVENT_LBUTTONDOWN:
#         r.append([x,y])
#         print("roi: ",r)
#         #if len(r) == 2:
#            # cv2.line(overlay,(r[0][0],r[0][1]),(r[1][0],r[1][0]),(0,0,255),1,cv2.LINE_AA)


# def test(degrees):
#     global encoderPrevDeg
#     quar = int(degrees) / 90
#     prevQuar = int(encoderPrevDeg) / 90
#     flip = 0
#     if (prevQuar - quar) < -1:
#         flip = -1
#     elif (prevQuar - quar) > 2:
#         flip = 1
#     else:
#         flip = 0
#
#     if flip > 0:
#         deldeg = (abs(encoderPrevDeg - 360) + abs(degrees)) * flip
#     elif flip < 0:
#         # deldeg = ((360 + (encoderPrevDeg - 360)) + abs(degrees)) * flip
#         deldeg = (encoderPrevDeg + (360 - degrees)) * flip
#     else:
#         deldeg = degrees - encoderPrevDeg
#
#     encoderPrevDeg = degrees
#     return deldeg
