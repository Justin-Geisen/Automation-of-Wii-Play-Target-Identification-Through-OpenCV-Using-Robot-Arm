# Note: moveL pose vector based on from "base" XYZ, RX, RY, RZ values on pendant IN METERS

import rtde_control
import rtde_receive
import rtde_io
import time
import paramiko
from math import pi
from math import atan
from math import sqrt

# robot arm network
rtde_c = rtde_control.RTDEControlInterface("192.168.1.102")
rtde_io_ = rtde_io.RTDEIOInterface("192.168.1.102")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.102")

# raspberry pi ssh
router_ip = "192.168.1.26"
router_username = "pi"
router_password = "pi"
ssh = paramiko.SSHClient()

# calibrate starting positions (bottom right corner of TV screen) if necessary (in radians)
originX = -pi/2 - 0.155
originY = -1.257

# AI lists
cursor = []
tList = []

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
            ssh.close()
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

    # the total movement (in radians) for each time this function is called
    totalMovement =  0.01

    # magnitude of vector
    magnitude = ( sqrt(xDif * xDif + yDif * yDif) )

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

def movToOrigin():
    joint_pos_new = [originX, -pi/2, -pi/2, -pi/2, 0, originY]
    rtde_c.moveJ(joint_pos_new, speed, acceleration)
        
if rtde_c.isConnected():
    print("RTDE is connected.")
    
    # max speed and acceleration is 3
    speed = 0.1
    acceleration = 0.1  
        
    movToOrigin();
    
    while 1:
        
        # demo/test results expected for cursor
        outputList = [23, 43]
        
        # set the variables
        getTargetList([ [32,23,25], [32, 23, 24]])
        getCursorCoor(outputList)
        
        # debug (can delete this)
        print (tList)
        
        # move and shoot
        movPos(cursor[0], cursor[1], tList[0][0], tList[0][1])
        shoot(cursor[0], cursor[1], tList[0][0], tList[0][1], tList[0][2])
        print ("global list:" + str(tList) )

    
    print(f"Finished moving.")
else:
    print("RTDE is not connected.")

rtde_c.stopScript()
