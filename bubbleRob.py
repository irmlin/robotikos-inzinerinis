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
    self.mapEnvironment = np.zeros((10000, 10000), dtype=int)
    self.robotOffsetPosInMap= (5000, 5000, 0)
    
    # Movement
    self.minMaxSpeed = [50*math.pi/180, 300*math.pi/180] # Min and max speeds for each motor
    self.tickFront = 2.5 # Tells whether bubbleRob is in forward or backward mode
    self.tickLeft = 0
    self.tickRight = 0
    self.mode = 0
    # -----------------------------------------------------------

    sim = require('sim')
    simUI = require('simUI')
    self.bubbleRobBase = sim.getObject('.') # this is bubbleRob's handle
    self.leftMotor = sim.getObject("./Left_Motor") # Handle of the left motor
    self.rightMotor = sim.getObject("./Right_Motor") # Handle of the right motor
    self.proxSensorFront = sim.getObject("./Prox_Front")
    self.proxSensorRight = sim.getObject("./Prox_Right")
    self.proxSensorLeft = sim.getObject("./Prox_Left")
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
    move()
    detect()

def move():
    #print(f"sim {sim.getSimulationTime()}")
    #print(f"forward {self.tickFront}")
    #print(f"right {self.tickRight}")
    #print(f"left {self.tickLeft}")
    
    if self.mode == 0:
        if sim.getSimulationTime() < self.tickFront:
            sim.setJointTargetVelocity(self.leftMotor, self.speed/1.5)
            sim.setJointTargetVelocity(self.rightMotor, self.speed/1.5)
        else:
            sim.setJointTargetVelocity(self.leftMotor, 0)
            sim.setJointTargetVelocity(self.rightMotor, 0)
            update()
    
    if self.mode == 1:
        # Turn right
        if sim.getSimulationTime() < self.tickRight:
            print("right")
            sim.setJointTargetVelocity(self.leftMotor, self.speed/2)
            sim.setJointTargetVelocity(self.rightMotor, -self.speed)
        else:
            sim.setJointTargetVelocity(self.leftMotor, 0)
            sim.setJointTargetVelocity(self.rightMotor, 0)
            update()
            
    if self.mode == 2:         
        # Turn left
        if sim.getSimulationTime() < self.tickLeft:
            print("left")
            sim.setJointTargetVelocity(self.leftMotor, -self.speed/2)
            sim.setJointTargetVelocity(self.rightMotor, self.speed)
        else:
            sim.setJointTargetVelocity(self.leftMotor, 0)
            sim.setJointTargetVelocity(self.rightMotor, 0)
            update()


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


def update():
    resultFront, *_ = sim.readProximitySensor(self.proxSensorFront) # Read the proximity sensor
    resultLeft, *_ = sim.readProximitySensor(self.proxSensorLeft) # Read the proximity sensor
    resultRight, *_ = sim.readProximitySensor(self.proxSensorRight) # Read the proximity sensor
    
    #print(f"front : {resultFront}")
    #print(f"left : {resultLeft}")
    #print(f"right : {resultRight}")
    
    if resultFront == 0:
        self.mode = 0
        self.tickFront = sim.getSimulationTime()+ 2.5
    else:
        if resultLeft == 1:
            self.mode = 1
            self.tickRight = sim.getSimulationTime()+ 1.75
        elif resultRight == 1:
            self.mode = 2
            self.tickLeft = sim.getSimulationTime()+ 1.75

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
