#python

import math
import numpy as np

def sysCall_init():
    # This is executed exactly once, the first time this script is executed
    
    # Custom added
    # -----------------------------------------------------------
    # Detection
    self.lastRobotCoords = None
    self.distanceXMoved = 0.0
    self.distanceYMoved = 0.0
    self.sampleRate = 0.1
    self.mapEnvironment = np.zeros((100000, 100000), dtype=int)
    self.robotOffsetPosInMap= (50000, 50000, 0)

    # Movement
    self.minMaxSpeed = [50*math.pi/180, 300*math.pi/180] # Min and max speeds for each motor
    self.mode = 0 # mode = 0: go forward; mode = 1: turn to new orientation; mode = 2: return to start
    self.RobotOrientation = 0.0 # orientation that the robot is supposed to face in degrees
    self.TriggerIR = 0 # Counter for reseting the IR sensor, it is also used to prevent the robot in geting stuck in forever left turn loops
    # -----------------------------------------------------------

    sim = require('sim')
    simUI = require('simUI')
    self.bubbleRobBase = sim.getObject('.') # this is bubbleRob's handle
    self.leftMotor = sim.getObject("./Left_Motor") # Handle of the left motor
    self.rightMotor = sim.getObject("./Right_Motor") # Handle of the right motor
    self.proxSensorFront = sim.getObject("./Prox_Front")
    self.proxSensorRight = sim.getObject("./Prox_Right")
    self.proxSensorLeft = sim.getObject("./Prox_Left")
    self.infraRedSensor = sim.getObject("./InfraRed")
    
    self.robotCollection = sim.createCollection(0)
    sim.addItemToCollection(self.robotCollection, sim.handle_tree, self.bubbleRobBase, 0)
    self.distanceSegment = sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, 1, [0, 1, 0])
    self.robotTrace = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, [1, 1, 0], None, None, [1, 1, 0])
    self.graph = sim.getObject('./Graph')
    # sim.destroyGraphCurve(self.graph, -1)
    
    self.distStream = sim.addGraphStream(self.graph, 'bubbleRob clearance', 'm', 0, [1, 0, 0])
    # Create the custom UI:
    xml = '<ui title="' + sim.getObjectAlias(self.bubbleRobBase, 1) + ' speed" closeable="false" resizeable="false" activate="false">'
    xml = xml + '<hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>'
    xml = xml + '<label text="" style="* {margin-left: 300px;}"/></ui>'
    self.ui = simUI.create(xml)
    self.speed = (self.minMaxSpeed[0] + self.minMaxSpeed[1]) * 0.5
    simUI.setSliderValue(self.ui, 1, 100 * (self.speed - self.minMaxSpeed[0]) / (self.minMaxSpeed[1] - self.minMaxSpeed[0]))

def sysCall_sensing():
    result, distData, *_ = sim.checkDistance(self.robotCollection, sim.handle_all)
    if result > 0:
        sim.addDrawingObjectItem(self.distanceSegment, None)
        sim.addDrawingObjectItem(self.distanceSegment, distData)
        sim.setGraphStreamValue(self.graph,self.distStream, distData[6])
    p = sim.getObjectPosition(self.bubbleRobBase)
    sim.addDrawingObjectItem(self.robotTrace, p)

def speedChange_callback(ui, id, newVal):
    self.speed = self.minMaxSpeed[0] + (self.minMaxSpeed[1] - self.minMaxSpeed[0]) * newVal / 100


def sysCall_actuation(): 
    if self.mode < 2:               # check if we are at finish, if not do usual routine
        if self.mode == 0:          
            move()
            detect()
            checkIfFinish()
        else:
            turn()
    else:
        restart()                   # if at the finish: comence shortest path search and return to start,
                                    # since this is not implemented yet restart the robot and let it go again

def move():
    resultFront, dist, *_ = sim.readProximitySensor(self.proxSensorFront) # Read the proximity sensor
    resultLeft, *_ = sim.readProximitySensor(self.proxSensorLeft) # Read the proximity sensor
    if (resultLeft == 0 and self.TriggerIR >= 25) or (resultFront == 1 and dist < 0.16):    # if robot can turn left or a wall is close enough in front
        sim.setJointTargetVelocity(self.leftMotor, 0)                                       # of the robot stop and start turning
        sim.setJointTargetVelocity(self.rightMotor, 0)
        self.TriggerIR = 0
        updateMevement()
    else:
        sim.setJointTargetVelocity(self.leftMotor, self.speed)
        sim.setJointTargetVelocity(self.rightMotor, self.speed)
        
    if np.abs(np.abs(getOrrientation()) - np.abs(self.RobotOrientation)) > 0.5: # if robots orintation strays to far away from needed, 
        self.mode = 1                                                           # go to turning mode for corrections

def turn():
    
    if self.RobotOrientation == 270.0:          # correct orientations since robot's orentiation can only be : (-180, 180)
        self.RobotOrientation = -90
    if self.RobotOrientation == -270.0:           
        self.RobotOrientation = 90.0
    if self.RobotOrientation == 180 and getOrrientation() < 0:     # if needed orientation is -180 or 180 degrees, set it to whichever one is closer
        self.RobotOrientation = -180                               # to the current orientation, since -180 == 180 in this simulation
    if self.RobotOrientation == -180 and getOrrientation() > 0:
        self.RobotOrientation = 180
    if np.abs(self.RobotOrientation) == 360:    # 360 and -360 degrees is 0 degrees in this simulation
        self.RobotOrientation = 0
        
    # print(f"new Orientation {self.RobotOrientation}, curr Orientation {getOrrientation()}")
    if np.abs(getOrrientation() - self.RobotOrientation) > 0.25:
        if np.abs(getOrrientation() - self.RobotOrientation)>350:
            sim.setJointTargetVelocity(self.leftMotor, -self.speed/4)                       
            sim.setJointTargetVelocity(self.rightMotor, self.speed/4)     # turn left, for correction to bottom 180 == -180
        elif self.RobotOrientation == 90 and getOrrientation() < -170:
            sim.setJointTargetVelocity(self.leftMotor, self.speed/4)      # turn right, because turning right is faster -180 -> 90
            sim.setJointTargetVelocity(self.rightMotor, -self.speed/4)
        elif self.RobotOrientation == -90 and getOrrientation() > 170:
            sim.setJointTargetVelocity(self.leftMotor, -self.speed/4)      # turn left, because turning left is faster 180 -> -90
            sim.setJointTargetVelocity(self.rightMotor, self.speed/4)
        elif getOrrientation() <  self.RobotOrientation:
            sim.setJointTargetVelocity(self.leftMotor, -self.speed/4)                       
            sim.setJointTargetVelocity(self.rightMotor, self.speed/4)     # turn left, because needed orientation is higher than current
        else:
            sim.setJointTargetVelocity(self.leftMotor, self.speed/4)      # turn right, because needed orientation is lower than current
            sim.setJointTargetVelocity(self.rightMotor, -self.speed/4)
    else:
        print(f"current orientation: {getOrrientation()}")
        sim.setJointTargetVelocity(self.leftMotor, 0)
        sim.setJointTargetVelocity(self.rightMotor, 0)
        self.mode = 0                                                   # if rotation is no longer needed return to "move forward" mode
        
def getOrrientation():
    return sim.getObjectOrientation(self.bubbleRobBase)[2] * 57.2957795 # return the orrientation of the robot in degrees

def updateMevement():
    resultFront, *_ = sim.readProximitySensor(self.proxSensorFront) # Read the proximity sensor
    resultLeft, *_ = sim.readProximitySensor(self.proxSensorLeft) # Read the proximity sensor
    resultRight, *_ = sim.readProximitySensor(self.proxSensorRight) # Read the proximity sensor
    self.mode = 1
    
    #print(f"front : {resultFront}")
    #print(f"left : {resultLeft}")
    #print(f"right : {resultRight}")
    
    if resultLeft == resultRight == resultFront == 1:           # if all proximity sensors are trigered, turn around
        print("turn around")
        self.RobotOrientation = self.RobotOrientation + 180.0   # turn to either side for 180 degrees
        return
        
    if resultLeft == 1:                                         # if the right proximity sensor is trigered, turn left
        print("turn right") 
        self.RobotOrientation = self.RobotOrientation - 90.0    # 90 degree rotation to the right is -90 degrees to curent orrientation
        return
    
    print("turn left")                                                  # if none of the other conditions are met, turn left
    self.RobotOrientation = self.RobotOrientation + 90.0                # this will triger if none of the side sensors are trigered, 
    return                                                              # or only the left sensor is trigered

    
def checkIfFinish():
    result = sim.readVisionSensor(self.infraRedSensor)
    sensorReading = False
    if isinstance(result, tuple):
        result, data, *_ = result
    if result >= 0:
        sensorReading = (data[10] < 0.5)  # Indexing adjusted for 0-based Python
    
    if not sensorReading and self.TriggerIR < 50:
        self.TriggerIR += 1
    if sensorReading and self.TriggerIR >= 5:
        print("returning to start")
        self.mode = 2

def restart():
    print("reached the end")                                # TODO implement 
    print("turning around and continuing traversal")
    self.mode = 1
    self.RobotOrientation = self.RobotOrientation + 180.0
    self.TriggerIR = 0
    
def detect():
    currentRobotCoords = sim.getObjectPosition(self.bubbleRobBase)
    if self.lastRobotCoords is None:
        self.lastRobotCoords = currentRobotCoords
        return
        
    self.distanceXMoved += abs(currentRobotCoords[0] - self.lastRobotCoords[0])
    self.distanceYMoved += abs(currentRobotCoords[1] - self.lastRobotCoords[1])
    self.lastRobotCoords = currentRobotCoords
    #print(f'x moved= {self.distanceXMoved}\ty moved= {self.distanceYMoved}')
    if self.distanceXMoved >= self.sampleRate or self.distanceYMoved >= self.sampleRate:
        # Add points to map
        sampleEnvironmentToMemory(currentRobotCoords)
        #print(f'x moved= {self.distanceXMoved}\ty moved= {self.distanceYMoved}')
        self.distanceXMoved = 0.0
        self.distanceYMoved = 0.0
    
    self.lastRobotCoords = currentRobotCoords

def sampleEnvironmentToMemory(currentRobotCoords):
    print('Sampling proximity!')
    print(f'Current robot coords: {currentRobotCoords}')
    # Read proximity sensors
    obstacleFront, distFront, *_ = sim.readProximitySensor(self.proxSensorFront)
    obstacleRight, distRight, *_ = sim.readProximitySensor(self.proxSensorRight)
    obstacleLeft, distLeft, *_ = sim.readProximitySensor(self.proxSensorLeft)
    
    # Read proximity sensors' coords
    coordsSensorFront = sim.getObjectPosition(self.proxSensorFront)
    coordsSensorRight = sim.getObjectPosition(self.proxSensorRight)
    coordsSensorLeft = sim.getObjectPosition(self.proxSensorLeft)
    
    # Direction vector for each sensor
    vFront = -1*  (np.array(coordsSensorFront) - np.array(currentRobotCoords)) / (np.linalg.norm(coordsSensorFront) - np.linalg.norm(currentRobotCoords))
    vRight = (np.array(coordsSensorRight) - np.array(currentRobotCoords)) / (np.linalg.norm(coordsSensorRight) - np.linalg.norm(currentRobotCoords))
    vLeft = -1* (np.array(coordsSensorLeft) - np.array(currentRobotCoords)) / (np.linalg.norm(coordsSensorLeft) - np.linalg.norm(currentRobotCoords))
    
    points = []
    if obstacleFront:
        p = coordsSensorFront + vFront * distFront
        print(f'Detected obstacle front! {p}')
        points.append(p)
    if obstacleRight:
        p = coordsSensorRight + vRight * distRight
        print(f'Detected obstacle right! {p}')
        points.append(p)
    if obstacleLeft:
        p = coordsSensorLeft + vLeft * distLeft
        print(f'Detected obstacle left! {p}')
        points.append(p)
    
    for point in points:
        pointPosInMap = [int(p * 1000 + self.robotOffsetPosInMap[i]) for i, p in enumerate(point)]
        self.mapEnvironment[pointPosInMap[1], pointPosInMap[0]] = 1
    #print(self.mapEnvironment)

def sysCall_cleanup(): 
    simUI.destroy(self.ui)
