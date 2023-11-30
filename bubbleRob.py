#python

import math
import numpy as np

def sysCall_init():
    # This is executed exactly once, the first time this script is executed
    sim = require('sim')
    simUI = require('simUI')
    self.bubbleRobBase = sim.getObject('.') # this is bubbleRob's handle
    self.leftMotor = sim.getObject("./leftMotor") # Handle of the left motor
    self.rightMotor = sim.getObject("./rightMotor") # Handle of the right motor
    self.noseSensorFront = sim.getObject("./sensingNoseFront")
    self.noseSensorRight = sim.getObject("./sensingNoseRight")
    self.noseSensorLeft = sim.getObject("./sensingNoseLeft")
    self.minMaxSpeed = [50*math.pi/180, 300*math.pi/180] # Min and max speeds for each motor
    self.backUntilTime = -1 # Tells whether bubbleRob is in forward or backward mode
    self.robotCollection = sim.createCollection(0)
    sim.addItemToCollection(self.robotCollection, sim.handle_tree, self.bubbleRobBase, 0)
    self.distanceSegment = sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, 1, [0, 1, 0])
    self.robotTrace = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, [1, 1, 0], None, None, [1, 1, 0])
    self.graph = sim.getObject('./graph')
    # sim.destroyGraphCurve(self.graph, -1)
    self.distStream = sim.addGraphStream(self.graph, 'bubbleRob clearance', 'm', 0, [1, 0, 0])
    # Create the custom UI:
    xml = '<ui title="' + sim.getObjectAlias(self.bubbleRobBase, 1) + ' speed" closeable="false" resizeable="false" activate="false">'
    xml = xml + '<hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>'
    xml = xml + '<label text="" style="* {margin-left: 300px;}"/></ui>'
    self.ui = simUI.create(xml)
    self.speed = (self.minMaxSpeed[0] + self.minMaxSpeed[1]) * 0.5
    simUI.setSliderValue(self.ui, 1, 100 * (self.speed - self.minMaxSpeed[0]) / (self.minMaxSpeed[1] - self.minMaxSpeed[0]))
    
    # Custom added
    # -----------------------------------------------------------
    self.lastRobotCoords = None
    self.distanceXMoved = 0.0
    self.distanceYMoved = 0.0
    self.sampleRate = 0.1
    self.mapEnvironment = np.zeros((10000, 10000), dtype=int)
    self.robotOffsetPosInMap= (5000, 5000, 0)
    # -----------------------------------------------------------

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
    currentRobotCoords = sim.getObjectPosition(self.bubbleRobBase)
    print(f'Robot current coords: {currentRobotCoords}')
    if self.lastRobotCoords is None:
        self.lastRobotCoords = currentRobotCoords
        return
        
    self.distanceXMoved += abs(currentRobotCoords[0] - self.lastRobotCoords[0])
    self.distanceYMoved += abs(currentRobotCoords[1] - self.lastRobotCoords[1])
    self.lastRobotCoords = currentRobotCoords
    #print(f'x moved= {self.distanceXMoved}\ty moved= {self.distanceYMoved}')
    if self.distanceXMoved >= self.sampleRate or self.distanceYMoved >= self.sampleRate:
        # Add points to map
        sampleEnvironmentToMemory()
        #print(f'x moved= {self.distanceXMoved}\ty moved= {self.distanceYMoved}')
        self.distanceXMoved = 0.0
        self.distanceYMoved = 0.0
    
    self.lastRobotCoords = currentRobotCoords

    if self.backUntilTime < sim.getSimulationTime():
        sim.setJointTargetVelocity(self.leftMotor, self.speed)
        sim.setJointTargetVelocity(self.rightMotor, self.speed)

    
    # If we detected something, we set the backward mode:s
    #if obstacleFront > 0:
    #    self.backUntilTime = sim.getSimulationTime() + 4
    #if self.backUntilTime < sim.getSimulationTime():
    #    # When in forward mode, we simply move forward at the desired speed
    #    sim.setJointTargetVelocity(self.leftMotor, self.speed)
    #    sim.setJointTargetVelocity(self.rightMotor, self.speed)
    #else:
    #    # When in backward mode, we simply backup in a curve at reduced speed
    #    sim.setJointTargetVelocity(self.leftMotor, -self.speed / 2)
    #    sim.setJointTargetVelocity(self.rightMotor, -self.speed / 8)

def sampleEnvironmentToMemory():
    print(f'Sampling proximity!')
    # Read proximity sensors
    obstacleFront, distFront, *_ = sim.readProximitySensor(self.noseSensorFront)
    obstacleRight, distRight, *_ = sim.readProximitySensor(self.noseSensorRight)
    obstacleLeft, distLeft, *_ = sim.readProximitySensor(self.noseSensorLeft)
    
    # Read proximity sensors' coords, because detected points coords are relative to sensors' coords
    coordsSensorFront = sim.getObjectPosition(self.noseSensorFront)
    coordsSensorRight = sim.getObjectPosition(self.noseSensorRight)
    coordsSensorLeft = sim.getObjectPosition(self.noseSensorLeft)
    
    # Get orientation of the sensors
    orientationSensorFront = sim.getObjectOrientation(self.noseSensorFront)
    orientationSensorRight = sim.getObjectOrientation(self.noseSensorRight)
    orientationSensorLeft = sim.getObjectOrientation(self.noseSensorLeft)
    
    points = []
    # We have: sensor world coords, sensor world orientation, distance from sensor to obstacle.
    # Here we calculate obstacle's coordinates using trigonometry
    if obstacleFront:
        x_abs = coordsSensorFront[0] + distFront * math.sin(orientationSensorFront[2])
        y_abs = coordsSensorFront[1] + distFront * math.cos(orientationSensorFront[2])
        z_abs = coordsSensorFront[2]
        p = [x_abs, y_abs, z_abs]
        print(f'Detected obstacle front! {p}')
        points.append(p)
    if obstacleRight:
        x_abs = coordsSensorRight[0] + distRight * math.sin(orientationSensorRight[2])
        y_abs = coordsSensorRight[1] + distRight * math.cos(orientationSensorRight[2])
        z_abs = coordsSensorRight[2]
        p = [x_abs, y_abs, z_abs]
        print(f'Detected obstacle right! {p}')
        points.append(p)
    if obstacleLeft:
        x_abs = coordsSensorLeft[0] + distLeft * math.sin(orientationSensorLeft[2])
        y_abs = coordsSensorLeft[1] + distLeft * math.cos(orientationSensorLeft[2])
        z_abs = coordsSensorLeft[2]
        p = [x_abs, y_abs, z_abs]
        print(f'Detected obstacle left! {p}')
        points.append(p)
    
    for point in points:
        pointPosInMap = [int(p * 1000 + self.robotOffsetPosInMap[i]) for i, p in enumerate(point)]
        self.mapEnvironment[pointPosInMap[1], pointPosInMap[0]] = 1
    #print(self.mapEnvironment)

def sysCall_cleanup(): 
    simUI.destroy(self.ui)
