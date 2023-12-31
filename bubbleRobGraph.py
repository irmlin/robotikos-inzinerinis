#python

import math
import numpy as np
import cv2
import heapq
import pickle
import networkx as nx
import matplotlib.pyplot as plt
import os

plt.switch_backend('Agg')

# Graph logic
class Node:
    def __init__(self, x, y, node_size=0.3):
        self.x = x
        self.y = y
        self.neighbours = []
        # A single node takes up (node_size x node_size) space,
        # where (x, y) is node center
        self.node_size = node_size 
        
    def distance(self, other_node):
        return math.sqrt((self.x - other_node.x)**2 + (self.y - other_node.y)**2)

    def is_same_node(self, other_node):
        """If other node is in the area around this node, it's the same node"""
        return abs(self.x - other_node.x) < self.node_size / 2 \
            and abs(self.y - other_node.y) < self.node_size / 2
            
    def add_neighbour(self, neighbour_id: int):
        self.neighbours.append(neighbour_id)
        
    def __repr__(self):
        return f"<Node x={self.x} y={self.y}>"
        
    def get_coords(self):
        return (self.x, self.y)


class MazeGraph:
    def __init__(self, output_dir):
        self.graph = {}
        # Robot's last visited node id
        self.last_active_node_id = None
        # Last inserted node id
        self.last_inserted_node_id = None
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
    def insert_node(self, node):
        # Check if node in this location does not already exist
        if len(self.graph) > 0:
            for node_id, graph_node in self.graph.items():
                if graph_node.is_same_node(node):
                    self.last_active_node_id = node_id
                    return
        
        if self.last_inserted_node_id is None:
            new_node_id = 0
        else:
            new_node_id = self.last_inserted_node_id + 1
        
        # Add new node to graph
        self.graph[new_node_id] = node
        # Mark neighbours
        if self.last_active_node_id is not None:
            self.graph[new_node_id].add_neighbour(self.last_active_node_id)
            self.graph[self.last_active_node_id].add_neighbour(new_node_id)
        self.last_active_node_id = new_node_id
        self.last_inserted_node_id =  new_node_id
        
    def get_last_active_node_id(self):
        return self.last_active_node_id
        
    def dijkstra(self, start_id, end_id):
        # Initialize distances and paths to all nodes
        distances = {node_id: float('inf') for node_id in self.graph}
        distances[start_id] = 0

        # Priority queue for nodes based on their distances
        priority_queue = [(0, start_id)]

        # Dictionary to store the shortest path to each node
        paths = {node_id: [] for node_id in self.graph}
        paths[start_id] = [start_id]

        while priority_queue:
            current_distance, current_node_id = heapq.heappop(priority_queue)
            # Check if the current node has already been visited
            if current_distance > distances[current_node_id]:
                continue

            current_node = self.graph[current_node_id]

            # Update distances and paths for neighbors
            for neighbor_id in current_node.neighbours:
                new_distance = distances[current_node_id] + current_node.distance(self.graph[neighbor_id])
                if new_distance < distances[neighbor_id]:
                    distances[neighbor_id] = new_distance
                    heapq.heappush(priority_queue, (new_distance, neighbor_id))
                    paths[neighbor_id] = paths[current_node_id] + [neighbor_id]
        
        # Return the distance and path to the end node
        return distances[end_id], paths[end_id]
        
    def __str__(self):
        # Print graph structure
        graph_str = ""
        for node_id in self.graph.keys():
            neighbours = self.graph[node_id].neighbours
            graph_str += f'Node_id: {node_id}, neighbours: {neighbours}\n'
        return graph_str
    
    def save_graph_to_file(self, shortest_path_nodes):
        shortest_path_nodes = [str(n) for n in shortest_path_nodes]
        adjadency_dict = {
            str(node_id):
                [str(neighbour_node_id) for neighbour_node_id in self.graph[node_id].neighbours]
                for node_id in self.graph
        }
        g = nx.Graph(adjadency_dict)
        #pos = nx.spring_layout(g)
        pos = {}
        for node_id in g.nodes:
            graph_node = self.graph[int(node_id)]
            x, y = graph_node.get_coords()
            x_pos, y_pos = int(x*1000), int(y*1000)
            pos[node_id] = (-y_pos, x_pos)
        node_colors = ['lightgreen' if node in shortest_path_nodes
            else 'lightblue' for node in g.nodes]
        
        nx.draw(g, pos=pos, node_color=node_colors, with_labels=True)
        # Color path lines and add arrows for direction
        edges = []
        for i in range(len(shortest_path_nodes)-1, 0, -1):
            node_a = shortest_path_nodes[i]
            node_b = shortest_path_nodes[i-1]
            edges.append((node_a, node_b))
        nx.draw_networkx_edges(g, pos=pos, edgelist=edges, edge_color='lightgreen',
            arrows=True, width=1,  arrowsize=20, arrowstyle='simple')
        
        output_path = os.path.join(self.output_dir, 'graph.jpg')
        plt.savefig(output_path)

def sysCall_init():
    # This is executed exactly once, the first time this script is executed
    
    # Custom added
    # -----------------------------------------------------------
    # Detection
    self.lastRobotCoords = None
    self.distanceXMoved = 0.0
    self.distanceYMoved = 0.0
    self.sampleRate = 0.01
    self.mapGraph = MazeGraph(output_dir='C:\\Users\\irman\\OneDrive\\Stalinis kompiuteris\\7\\robotika\\robotikos-inzinerinis\\results')
    self.startNodeId = None
    self.finishNodeId = None

    # Movement
    self.minMaxSpeed = [50*math.pi/180, 300*math.pi/180] # Min and max speeds for each motor
    self.mode = 0 # mode = 0: go forward; mode = 1: turn to new orientation; mode = 2: return to start; mode = 3: move_back; mode =4: turn_back; mode =5 stop
    self.RobotOrientation = 0.0 # orientation that the robot is supposed to face in degrees
    self.TriggerIR = 0 # Counter for reseting the IR sensor, it is also used to prevent the robot in geting stuck in forever left turn loops
    
    self.PathBack = []
    self.currPathbackNode = 0
    self.nextIsEnd = False
    self.currDestination = 0
    self.dirr = 0
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
    if self.mode == 0:
        move()
        measureMovedDistance()
        checkIfFinish()             # check if we are at finish, if not do usual routine
    elif self.mode == 1:
        turn(0)
    elif self.mode == 2:
        returnToStart()             # if at the finish: comence shortest path search and return to start
    elif self.mode == 3:
        back_move()
    elif self.mode == 4:
        turn(3)
    else:
        print("returned to start")
        

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

def back_move():
    currLocation = sim.getObjectPosition(self.bubbleRobBase)[:2]
    if np.abs(currLocation[self.dirr] - self.currDestination[self.dirr]) <0.05:
        sim.setJointTargetVelocity(self.leftMotor, 0)                                       
        sim.setJointTargetVelocity(self.rightMotor, 0)
        self.currPathbackNode += 1
        back_update()
    else:
        sim.setJointTargetVelocity(self.leftMotor, self.speed)
        sim.setJointTargetVelocity(self.rightMotor, self.speed)
    
    if np.abs(np.abs(getOrrientation()) - np.abs(self.RobotOrientation)) > 0.5: # if robots orintation strays to far away from needed, 
        self.mode = 4  
    
def back_update():
    
    if self.nextIsEnd:
        self.mode = 5
    else:
        n1 = self.PathBack[self.currPathbackNode]
        n2 = self.PathBack[self.currPathbackNode + 1]
        
        if n2 == self.startNodeId:
            self.nextIsEnd = True
        
        coords1 = self.mapGraph.graph[n1].get_coords()
        coords2 = self.mapGraph.graph[n2].get_coords()
        self.currDestination = coords2
        print(f"moving to {self.currDestination} from node {n1} to node {n2}")
                                                                                   # +
        x_dif = coords1[0] - coords2[0]                                            # x /\ , y +<>-
        y_dif = coords1[1] - coords2[1]                                            # - \/
        
        newOrientation = 0
        if np.abs(x_dif) > np.abs(y_dif):
            if x_dif > 0:
                newOrientation = -180
            else:
                newOrientation = 0
            self.dirr = 0
        else:
            if y_dif > 0:
                newOrientation = -90
            else:
                newOrientation = 90
            self.dirr = 1
        
        if newOrientation == self.RobotOrientation:
            self.mode = 3
        elif np.abs(newOrientation) == 180 and np.abs(self.RobotOrientation) == 180:
            self.mode = 3
        else:
            self.RobotOrientation = newOrientation
            self.mode = 4
    
def turn(prev_mode):
    
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
        self.mode = prev_mode                                              # if rotation is no longer needed return to "move forward" mode
        
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

def calculateShortestDistance():
    self.finishNodeId = self.mapGraph.get_last_active_node_id()
    if self.startNodeId is None:
        self.startNodeId = 0
    
    distance, path = self.mapGraph.dijkstra(self.startNodeId, self.finishNodeId)
    print(f'Shortest distance: {distance}, path: {path}')
    return distance, path
    
def returnToStart():
    sim.setJointTargetVelocity(self.leftMotor, 0)                                       
    sim.setJointTargetVelocity(self.rightMotor, 0)
    distance, path = calculateShortestDistance()
    self.mapGraph.save_graph_to_file(shortest_path_nodes=path)
    self.PathBack = np.flip(np.array(path))
    back_update()

#deprecated
def restart():
    print("reached the end")                                # TODO implement 
    print("turning around and continuing traversal")
    self.mode = 1
    self.RobotOrientation = self.RobotOrientation + 180.0
    self.TriggerIR = 0
    
def measureMovedDistance():
    currentRobotCoords = sim.getObjectPosition(self.bubbleRobBase)
    if self.lastRobotCoords is None:
        self.lastRobotCoords = currentRobotCoords
        # Insert start node to graph
        sampleEnvironmentToMemory(currentRobotCoords)
        self.startNodeId = 0
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
    node = Node(currentRobotCoords[0], currentRobotCoords[1])
    self.mapGraph.insert_node(node)

def sysCall_cleanup(): 
    simUI.destroy(self.ui)