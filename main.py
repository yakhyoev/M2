print("test main.py")

import time
start_time = time.perf_counter()

def stamp(msg):
    print(f"{msg}: {time.perf_counter() - start_time:.3f}s")

stamp('start')

import os  #temporary for testing?
# import socket
import serial
# import threading
import select
import sys
# import queue
# import matplotlib.pyplot as plt
print("importing time, math")
# import time
import math
# print("importing np")
stamp('import ser, sel, sys, math, os')

import numpy as np
stamp('import numpy')

# print("importing cv2")
import cv2
stamp('import cv2')

test = False #True
angle = 3
lower = [50, 70, 70]
u = [100, 255, 255]
g = 7
e = 66600
r = []
share = False
shareData = ""
tcp = False
cam = object
deltaX = "0"
deltaY = "0"

# add - algo to only select the botton edge; compare edge to the centroid and pick the one with lower y
# add - take test shots after micro adjustment; label
# add - special code for 12 port
# add - command to reset rpi from arduino


def run(cam, portName="port", portNumber = 2):
    global angle, lower, u, g, prev, e, share, shareData, deltaX, deltaY
    cxLeftmost = 0  # x position of centroid
    verticalLineTarget = 120
    horizontalLineTarget = 185

    portColumn = portNumber % 12

    # stamp('before camstart in run')
    # cam.start()
    # stamp('after cam start')
    try:
        if not test:
            frame = cam.capture_array()
            frame = frame[105:330, :150]


        else:  #if testing
            if len(imgFiles) > 0:
                if share:
                    imgPath = "/Users/saidyakhyoev/rpi open cv folder/snapshots/" + imgFiles.pop()
                    print(imgPath)
                    share = False

            frame = cv2.imread(imgPath)  # 81,164

        # Rotate to make the image level
        (h, w) = frame.shape[:2]
        center = (w // 2, h // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_image = cv2.warpAffine(frame, rotation_matrix, (w, h))

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
                if cv2.contourArea(ct) > 100:  # adjust 100 to the minimum desired area.
                    M = cv2.moments(ct)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        centroidAggregatorY.append(cy)
                        centroidAggregatorX.append(cx)
                        cv2.drawMarker(overlay, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10)

            if len(centroidAggregatorY) > 0:
                cyMean = int(sum(centroidAggregatorY) / len(centroidAggregatorY))
                cv2.drawMarker(overlay, (cx, cyMean), (0, 255, 255), markerType=cv2.MARKER_DIAMOND, markerSize=10)
                # if share:
                #     print("shared")
                #     # shareData = cyMean
                #     # send_queue.put("centroid y "+str(cyMean))
                #
                #     share = False

            if len(centroidAggregatorX) > 0:
                cxLeftmost = sorted(centroidAggregatorX)[0]
                if portColumn == 0:
                    cxLeftmost = sorted(centroidAggregatorX)[-1]
                cv2.drawMarker(overlay, (cxLeftmost, cyMean), (0, 0, 255), markerType=cv2.MARKER_SQUARE,
                               markerSize=10)
                # send_queue.put("left blob centroid x " + str(cxLeftmost[0]))

            # crop image
            # if len(r) == 2:
            #     frame = frame[r[0][1]:r[1][1], r[0][0]:r[1][0]]
            # if cxLeftmost > 0:
            #     mask = mask[:cyMean + 85, : cxLeftmost + 75]
            #     overlay = overlay[:cyMean + 85, : cxLeftmost + 75]  # crop image around left connector
            # else:
            #     mask = mask  # [100:300, 20:300]  #[130:310, :110] # picamera [y,x]
            #     overlay = overlay
            mask = mask[10:300, :130]  # picamera [y,x]  [130:310, :110]
            overlay = overlay[10:300, :130]

            #Detect edges in the cropped mask
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)

        # scale line
        cv2.line(overlay, (verticalLineTarget, 0), (verticalLineTarget, 225), (255, 255, 255), 1, cv2.LINE_AA)  # vertical target line
        cv2.line(overlay, (0, horizontalLineTarget), (175, horizontalLineTarget), (255, 255, 255), 1, cv2.LINE_AA)  # horizontal target line

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
                cv2.line(overlay, (rightmostLine[0], rightmostLine[1]), (rightmostLine[2], rightmostLine[3]), (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"R:{rightmostLine[2]}", (5, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255), 1)
                cv2.putText(overlay, f"dx :{verticalLineTarget - rightmostLine[2]}", (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 0, 255), 1)
                deltaX = rightmostLine[2]-verticalLineTarget  # x target 73
                # if portColumn == 0:
                #     deltaX += 40  # shift camera to the right, est. widge of connector
                deltaX = str(deltaX) + "\n"
                ser.write(deltaX.encode())

            # Bottom line
            if len(horizontalLines) > 0:
                # eliminate lines that are above the centroid, since we need the bottom edge of the connector
                horizontalLines = [x for x in horizontalLines if x[3] > cyMean]
                if len(horizontalLines) > 0:
                    horizontalLines = sorted(horizontalLines, key=lambda x: x[3])
                    print("horizontal lines, length:", horizontalLines)
                    bottomLine = horizontalLines[-1]
                else:
                    bottomLine = [0,175,52,175]  # if no lower edge found, substitute artificial line

                print("bottomLine", bottomLine)
                print("cyMean", cyMean)
                cv2.line(overlay, (bottomLine[0], bottomLine[1]), (bottomLine[2], bottomLine[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"B:{bottomLine[3]}", (5, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                cv2.putText(overlay, f"dy:{170 - bottomLine[3]}", (5, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                if bottomLine[3]:
                    deltaY = str(bottomLine[3]-horizontalLineTarget)
                    deltaY = deltaY + "\n"
                    ser.write(deltaY.encode())
                else:
                    deltaY = (str(0)+"\n").encode()
                    ser.write(deltaY)

            # Display the frame (optional)
        # cv2.imshow("frame",frame)
        cv2.imshow("final overlay", overlay)
        # time.sleep(5)
        # cv2.setMouseCallback("overlay", click_event)
        name = "snapshots/"+portName+".jpg"
        print(name, "\n", deltaX, deltaY)
        cv2.imwrite(name, overlay)
        # cv2.imwrite(name+"frame.jpg", frame)
        # time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        if not test:
            pass
            # cam.stop()
        # cv2.destroyAllWindows()
        # s.close()


# MAIN CODE
print("main code")

if test:  # load images from folder
    imgFiles = os.listdir("/Users/saidyakhyoev/rpi open cv folder/snapshots/")
    imgPath = "/Users/saidyakhyoev/rpi open cv folder/snapshots/snapshot.jpg"
else:
    try:
        print("importing picamera2")
        stamp('before picam')
        from picamera2 import Picamera2
        stamp('import picam')

        # Serial comm
        ser = serial.Serial('/dev/ttyS0', 9600, timeout=1) #open serial port
        ser.write(b'RPi ready\n')  # send string to Arduino

        # setup camera
        cam = Picamera2()
        stamp('cam=Picamera2')

        configRGB = cam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
        cam.configure(configRGB)
        cam.set_controls({'ExposureTime': e, 'AnalogueGain': g, 'AeEnable': False})
        print("rebooting ardu")
        # ser.write(b': : {"commandType":10}')
        ser.write(b'home')
        ser.write(b'home')
        print("ready v3-typable commands")
        typed = ""
        stamp('after cam config')

        cam.start() #
        time.sleep(0.1) #

        while True:
            img = cam.capture_array()  #
            img = img[105:330, :150]  #
            cv2.imshow("test", img)  # I do not see real time image as expect here
            cv2.waitKey(1)

            if ser.in_waiting >= 5:
                msg = ser.readline().decode('utf-8').rstrip()
                print("ardu: ", msg)

                if "CHK" in msg:
                    portName = msg[3:]
                    portNumber = int(portName)
                    print("CHK", portName, portNumber)
                    run(cam, portName, portNumber)  # take pic and find port misalignment in pixels

                elif "COR" in msg:
                    portName = msg[3:]
                    portNumber = int(portName)
                    print("COR", "cor-"+portName, portNumber)
                    run(cam, "cor-"+portName, portNumber)

            # read user input
            if select.select([sys.stdin], [], [], 0.0)[0]:  # a tuble of 3 lists: readable, writable, error. use first, readabe
                line = sys.stdin.readline().strip().encode()

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
        print("Serial port closed.")

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
