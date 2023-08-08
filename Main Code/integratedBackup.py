# Note: moveL pose vector based on from "base" XYZ, RX, RY, RZ values on pendant IN METERS

# robot imports
import rtde_control
import rtde_receive
import rtde_io
import time
import paramiko
from math import pi
from math import atan
from math import sqrt

# AI imports
import cv2 as cv
import numpy as np
import math
import time
import itertools as it

# robot arm network
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")
rtde_io_ = rtde_io.RTDEIOInterface("192.168.1.102")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")

# raspberry pi ssh
router_ip = "192.168.1.26"
router_username = "pi"
router_password = "pi"
ssh = paramiko.SSHClient()



# robot object lists
cursor = []
tList = []

# used to restart game
start_time = 0

# used to move side to side
i = 0
movCount = 0

# AI functions

def findCursorTM(frame, template):
    width = int(template.shape[1]/3)
    height = int(template.shape[0]/3)
    template = cv.resize(template, (width, height), interpolation=cv.INTER_NEAREST_EXACT)
    grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    grayTemplate = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    templateW, templateH = grayTemplate.shape[::-1]
    templateRes = cv.matchTemplate(grayFrame, grayTemplate, cv.TM_CCOEFF_NORMED)
    # print('template matching...')
    _, maxVal, _, max_loc = cv.minMaxLoc(templateRes)
    # print(maxVal)
    return maxVal, (max_loc[0], max_loc[1], templateW, templateH)



def detectObj(frame):
    # roi_frame = frame[ 0:715, 0:1920 ]
    roi_frame = frame[ 0:320, 0:640 ]

    mask = obj_dectection.apply(roi_frame)
    _, mask = cv.threshold(mask, 254, 255, cv.THRESH_BINARY)

    dilatedImg = cv.dilate(mask, (69,69), iterations=1)
    erodedImg = cv.erode(dilatedImg, (121,121), iterations=15)
    dilatedImg = cv.dilate(erodedImg, (5,5), iterations=5)

    contours, _ = cv.findContours(dilatedImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    detectedContours = []

    for count in contours: 
        cv.drawContours(frame, [count], -1, (0,0,255))
        area = cv.contourArea(count)
        # print(area)
        if area > 500:
            x, y, w, h = cv.boundingRect(count)
            topLeft = (x, y)
            bottomRight = ( x + w, y + h)
            center = ( (topLeft[0]+bottomRight[0])/2, (topLeft[1]+bottomRight[1])/2 )

            cv.rectangle(frame, topLeft, bottomRight, (255,0,0), 3)
            cv.drawMarker(frame, (int(center[0]),int(center[1])), (255,0,0))
            detectedContours.append((int(center[0]),int(center[1]), int(w/2)))
    
    # cv.imshow('test', mask)
    return detectedContours

def drawLine( detectedContours, index = 0, color = (0,0,0) ):
    if not detectedContours or len(detectedContours) == 1:
        return None
    
    next_idx = index + 1
    try:
        if detectedContours[next_idx]:
            currPos = (detectedContours[index][0], detectedContours[index][1])
            nextPos = (detectedContours[next_idx][0], detectedContours[next_idx][1])
            cv.line(frame, currPos, nextPos, color, 2)
            return drawLine(detectedContours, index + 1, color)
    except:
        return None

def drawCursor(frame, cursorBox):
    cBoxTL = (int(cursorBox[0]), int(cursorBox[1]))
    cBoxBR = (int(cursorBox[0] + cursorBox[2]), int(cursorBox[1] + cursorBox[3]))
    cBoxCenter = (int((cBoxTL[0] + cBoxBR[0])/2), int((cBoxTL[1] + cBoxBR[1])/2))
    cv.circle(frame, cBoxCenter, int(cursorBox[2]/4), (0,255,0), 4)
    cv.drawMarker(frame, (cBoxCenter[0], cBoxCenter[1]), (0,255,0), cv.MARKER_TILTED_CROSS, 25, 4)
    return cBoxCenter

def dist(x,y):
    return math.hypot(y[0]-x[0],y[1]-x[1])

def sortedShortestPath(detectedContours):
    paths = [ p for p in it.permutations(detectedContours) ]
    path_distances = [ sum(map(lambda x: dist(x[0],x[1]), zip(p[:-1],p[1:]))) for p in paths ]
    min_index = np.argmin(path_distances)
    return paths[min_index]

# robot functions

def getCursorCoor(cursorList):
    global cursor
    cursor = cursorList.copy()

def getTargetList(listOfTargets):
    global tList 
    tList = listOfTargets.copy()
    #print(tList)
 #   i = 0
 #   for x in listOfTargets:
 #       j = 0
  #      for y in x:
  #          tList[i][j] = y
  #          j = j+1        
  #      i = i+1
        


def wiiCommandInit(command):
    """ Connect to a device, run a command, and return the output."""

    # Load SSH host keys.
    ssh.load_system_host_keys()
    # Add SSH host key when missing.
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    total_attempts = 3
    for attempt in range(total_attempts):
        try:
            print("Attempt to connect: %s" % attempt)
            # Connect to router using username/password authentication.
            ssh.connect(router_ip, 
                        username=router_username, 
                        password=router_password,
                        look_for_keys=False )
            # Run command.
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(command)
            # Read output from command.
            output = ssh_stdout.readlines()
            print("test")
            # Close connection.
            #ssh.close()
            return output

        except Exception as error_message:
            print("Unable to connect")
            print(error_message)

def wiiCommand(command):
    """ Connect to a device, run a command, and return the output."""

    # Load SSH host keys.
    ssh.load_system_host_keys()
    # Add SSH host key when missing.
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    total_attempts = 3
    for attempt in range(total_attempts):
        try:
            print("Attempt to connect: %s" % attempt)
            # Connect to router using username/password authentication.
            #ssh.connect(router_ip, 
            #            username=router_username, 
            #            password=router_password,
            #            look_for_keys=False )
            # Run command.
            ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(command)
            # Read output from command.
            output = ssh_stdout.readlines()
            print("test")
            # Close connection.
            #ssh.close()
            return output

        except Exception as error_message:
            print("Unable to connect")
            print(error_message)


def shoot(xCursor, yCursor, xTarget, yTarget, radius):
    global tList
    xDif = xTarget - xCursor
    yDif = yTarget - yCursor
    
    distance = (sqrt(xDif*xDif + yDif*yDif))
    
    if(distance <= radius):
        wiiCommand("python3 /home/pi/Desktop/trigger.py")
        print("shot coordinates")
        #tList.pop(0)
        #print("Modified list is : " + str(tList))
        
def movPos(xCursor, yCursor, xTarget, yTarget):
    xDif = xTarget - xCursor
    yDif = yTarget - yCursor
    yDif *= -1

    # the total movement (in radians) for each time this function is called
    totalMovement =  0.013

    # magnitude of vector
    magnitude = ( sqrt(xDif * xDif + yDif * yDif) )

    print( "<" + str(xDif/magnitude) + ", " + str(yDif/magnitude) + ">")

    # change vector
    xChange = xDif / magnitude * totalMovement
    yChange = yDif / magnitude * totalMovement

    # find actual physical current position so that we can add it the relative movement change
    currentJointPos = rtde_r.getActualQ()

    # calculate new position
    xNewPos = currentJointPos[0] - xChange
    yNewPos = currentJointPos[5] + yChange


    # create the new joint with the new positions
    newJointPos = [ xNewPos, currentJointPos[1], currentJointPos[2], currentJointPos[3], currentJointPos[4], yNewPos ]

    # move to new position
    rtde_c.moveJ(newJointPos, speed, acceleration)

def movToOrigin(originX, originY):
    joint_pos_new = [originX, -pi/2, -pi/2, -pi/2, 0, originY]
    rtde_c.moveJ(joint_pos_new, speed, acceleration)

def degToRad(angle):
    return angle / 180 * pi

def restartGame(current_time):
    global start_time
    if current_time - start_time >= 210:

        # hover over the "Play Again" button
        newJointPos = [ -pi/2 - 0.11, -pi/2, -pi/2, -pi/2, 0, -1.176 ]
        rtde_c.moveJ(newJointPos, speed, acceleration)

        # press "A" to select the button
        wiiCommand("python3 /home/pi/Desktop/a.py")

        # restart timer
        start_time = time.time()

def movLeft(pos):
    global i
    newJointPos = [ degToRad(pos), -pi/2, -pi/2, -pi/2, 0, degToRad(-68) ]
    rtde_c.moveJ(newJointPos, speed, acceleration)
    #wiiCommand("python3 /home/pi/Desktop/trigger.py")

    i += 0.2

def movRight(pos):
    global i
    newJointPos = [ degToRad(pos), -pi/2, -pi/2, -pi/2, 0, degToRad(-68) ]
    rtde_c.moveJ(newJointPos, speed, acceleration)
    #wiiCommand("python3 /home/pi/Desktop/trigger.py")

    i -= 0.2     

def movNew( xTargetPx, yTargetPx ):    
    global start_time

    # 2 thirds of the screen is where the bullseyes appear
    screenHeight = 480/3 * 2

    if time.time() - start_time <= 35:
        yTargetPx -= 100
    else: 
        # top third
        if yTargetPx < 480/3:
            yTargetPx += 40

        # bottom third
        elif yTargetPx > screenHeight/3 * 2:
            yTargetPx -= 30
            print("bottom third")

    # coordinate of target in radians
    targetX = degToRad( -(10.33/640 * xTargetPx) - 92.17 )
    targetY = degToRad( -(11.13/480 * yTargetPx) - 39.37 )

    # move to target
    newJointPos = [ targetX, -pi/2, -pi/2, -pi/2, 0, targetY ]
    rtde_c.moveJ(newJointPos, speed, acceleration)

    




# capture live footage
# vid = cv.VideoCapture( 1 )

###### MAIN LOOP ######

# vid = cv.VideoCapture('game_footage_1.mkv')
# vid = cv.VideoCapture('game_footage_2.mkv')
# vid = cv.VideoCapture('game_footage_3.mkv')
vid = cv.VideoCapture('/dev/video0') #640x480

#template = cv.imread('cursor.png')

# background subtraction parameter (change to get better detection)
bgHistory = 90
bgVarThreshold = 400
bgDetectShadow = False
obj_dectection = cv.createBackgroundSubtractorMOG2(bgHistory, bgVarThreshold, bgDetectShadow)

#tracker = cv.TrackerCSRT_create()

# fps counter
prev_frame_time = 0
new_frame_time = 0
frame_count = 0

isCursorFound = True
returnSuccc = True

# calibrate starting positions (bottom right corner of TV screen) if necessary (in radians)
originX = degToRad(-97.35)
originY = degToRad(-43.45)
        
if rtde_c.isConnected():
    print("RTDE is connected.")
    
    # max speed and acceleration is 3
    speed = 3
    acceleration = 3
        
    movToOrigin(originX, originY)
    
    wiiCommandInit("python3 /home/pi/Desktop/a.py")
    start_time = time.time()

    i = -100.5

    movNew(160,240)

    while(True):           


        # this restarts the game
        #restartGame(time.time())
        # if time.time() - start_time >= 210:

        #     # hover over the "Play Again" button
        #     newJointPos = [ -pi/2 - 0.11, -pi/2, -pi/2, -pi/2, 0, -1.176 ]
        #     rtde_c.moveJ(newJointPos, speed, acceleration)

        #     # press "A" to select the button
        #     wiiCommand("python3 /home/pi/Desktop/a.py")

        #     # restart timer
        #     start_time = time.time()

        # move side to side
        #if movCount >= 0 and movCount < 40:
            #newJointPos = [ degToRad(i), -pi/2, -pi/2, -pi/2, 0, degToRad(-68) ]
            #rtde_c.moveJ(newJointPos, speed, acceleration)
            #wiiCommand("python3 /home/pi/Desktop/trigger.py")

            #i += 0.2
            #movLeft(i)
            #movCount += 1
        #elif movCount >= 40 and movCount < 80:
            #newJointPos = [ degToRad(i), -pi/2, -pi/2, -pi/2, 0, degToRad(-68) ]
            #rtde_c.moveJ(newJointPos, speed, acceleration)
            #wiiCommand("python3 /home/pi/Desktop/trigger.py")

            #i -= 0.2
            #movRight(i)
            #movCount += 1
        #else:
            #movCount = 0
        # while i < -91.5:
        #     newJointPos = [ degToRad(i), -pi/2, -pi/2, -pi/2, 0, degidegToRad(-68) ]
        #     rtde_c.moveJ(newJointPos, speed, acceleration)
        #     wiiCommand("python3 /home/pi/Desktop/trigger.py")

        #     i -= 0.2
        

        # Capture the video frameq
        ret, frame = vid.read()
        # print(ret)
        # if not ret:
        #     break
        # find the Cursor
        #if not isCursorFound:
            #cursorVal, cursorRectangle = findCursorTM(frame, template)
            # print(cursorVal)
            #if cursorVal >= .285:
                # print(cursorVal)
                #tracker.init(frame, cursorRectangle)
                #isCursorFound = True
        #else: 
            #returnSuccc, cursorBox = tracker.update(frame)

        sortedDetectedContours = []
        cursorCenter = []

        #if returnSuccc and isCursorFound:
            #cursorCenter = drawCursor(frame, cursorBox)
            # print(cursorCenter)
            #cv.putText(frame, "Tracking cursor", (80,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,255,0),2)
        #else:
            #cv.putText(frame, "Tracking cursor failure", (80,80), cv.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        #if frame_count % 40 == 0:
            #isCursorFound = False
            #frame_count = 0

        # TODO: broken code, need to find solutions
        if isCursorFound and returnSuccc: 
            # This is the area that we are interested in using
            detectedContours = detectObj(frame)

            # This sorts the list of detected contours
            sortedDetectedContours = sortedShortestPath(detectedContours)

            # print(detectedContours)
            drawLine(sortedDetectedContours, color=(255,255, 0))



        # robot control
        # demo/test results expected for cursor
        # outputList = [23, 43]

        
        # TODO: fix drawCursor: robot team commented out following lines
        #cursorBox = tracker.update(frame)
        # set the variables
        #print(cursorCenter)
        #print(sortedDetectedContours)
        #if len(cursorCenter) > 0 and len(sortedDetectedContours) > 0 and frame_count % 8 == 0:
        # move and shoot
            #movPos(cursorCenter[0], cursorCenter[1], sortedDetectedContours[0][0], sortedDetectedContours[0][1])
            #wiiCommand("python3 /home/pi/Desktop/trigger.py")
            #shoot(cursorCenter[0], cursorCenter[1], sortedDetectedContours[0][0], sortedDetectedContours[0][1], sortedDetectedContours[0][2])
        #print ("global list:" + str(tList) )
        if len(sortedDetectedContours) > 0 and frame_count % 8 == 0:
        # move and shoot
            movNew(sortedDetectedContours[0][0], sortedDetectedContours[0][1])
            wiiCommand("python3 /home/pi/Desktop/trigger.py")
            #shoot(cursorCenter[0], cursorCenter[1], sortedDetectedContours[0][0], sortedDetectedContours[0][1], sortedDetectedContours[0][2])

        # fps counter
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time
        fps = str(int(fps))
        cv.putText(frame, fps, (7, 70), cv.FONT_HERSHEY_COMPLEX_SMALL, 3, (0, 0, 255), 3, cv.LINE_AA)

        # Display the resulting frame
        cv.imshow('frame', frame)
        # cv.imshow('frame roi', roi_frame)
        # cv.imshow('bitWiseOp', bitwiseOp)
        # cv.imshow('fg mask', mask)
        # cv.imshow('dialate', dilatedImg)
        # cv.imshow('eroded', erodedImg)
        
        frame_count = frame_count + 1
        # set 'q' as quit
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv.destroyAllWindows()

    print(f"Finished moving.")
else:
    print("RTDE is not connected.")

rtde_c.stopScript()
