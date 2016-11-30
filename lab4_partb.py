#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math
import heapq

class node(object):
    def __init__(self):
	
	self.parent = None
	self.pos= Point()
	self.orientation = 0
	self.g = 0
class PQ(object):
    def __init__(self):
	self.dist = []
	self.cells = GridCells()
	self.goal = Point()
	self.start= Point()
	self.cells.header.frame_id = 'map'
	self.cells.cell_width = resolution 
	self.cells.cell_height = resolution
    def empty(self):
	return len(self.dist) == 0
    def push(self, node):
        global frontier
	heapq.heappush(self.dist, (heuristic(node.pos,self.goal),node))
	self.cells.cells.append(node.pos)
	frontier.cells.append(node.pos)
    def pop(self):
	global frontier
	frontier.cells.remove(self.dist[0][1].pos)
	self.cells.cells.remove(self.dist[0][1].pos)
	
	return heapq.heappop(self.dist)
  


# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print "mapCallBack"
    print data.info
    

def readGoal(goal):
    global goalX
    global goalY
    global goalP
    global goalFlag
    global onepath
    
    goalP = Point()
    goalFlag = True
    
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    
    goalP.x = goalX
    goalP.y = goalY
    goalP.z = 0
    
    # print "readGoal"
    # print goal.pose
    
    goalFlag = True
    onepath = True
    
    
def readStart(startPos):

    global startPosX
    global startPosY
    global startP
    global startFlag
    global onepath
    
    startP = Point()
    startFlag = True
    
    # startPosX = startPos.pose.pose.position.x
    # startPosY = startPos.pose.pose.position.y
    
    startP.x = startPos.position.x
    startP.y = startPos.position.y
    startP.z = 0
    
    # print "readStart"
    # print startPos.pose.pose
    
    startFlag = True
    onepath = True
    
    
def heuristic(current, goal):
    return abs(current.x-goal.x)+abs(current.y-goal.y)
    # return math.sqrt(math.pow((current.x-goal.x),2)+math.pow((current.y-goal.y),2))

def point2Cell(point):
    pointreturn = Point()
    pointreturn.x = int((point.x-offsetX - (0.5 * resolution))/resolution)
    pointreturn.y = int((point.y-offsetY + (0.5 * resolution))/resolution)    
    
    pointreturn.z = (pointreturn.y-1)*width+pointreturn.x
  
    point=Point()
    pointreturn.x=(pointreturn.x*resolution)+offsetX + (0.5 * resolution) # added secondary offset 
    pointreturn.y=(pointreturn.y*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?

    

    return pointreturn




def createNeighbor(point,PQ):
    global path
    global frontier
    global cells
    W = Point(point.pos.x,point.pos.y+resolution,point.pos.z)
    A = Point(point.pos.x-resolution,point.pos.y,point.pos.z)
    D = Point(point.pos.x+resolution,point.pos.y,point.pos.z)
    S = Point(point.pos.x,point.pos.y-resolution,point.pos.z)
    
    if cells.cells.count(W) == 0 and path.cells.count(W) == 0 and frontier.cells.count(W) == 0:
      wnode = node()
      wnode.parent = point
      wnode.g = point.g + 1
      wnode.orientation = 1
      wnode.pos = W
      PQ.push(wnode)
    if cells.cells.count(A) == 0 and path.cells.count(A) == 0 and frontier.cells.count(A) == 0:
      anode = node()
      anode.parent = point
      anode.pos = A
      anode.g = point.g + 1
      anode.orientation = 2
      PQ.push(anode)
    if cells.cells.count(S) == 0 and path.cells.count(S) == 0 and frontier.cells.count(S) == 0:
      snode = node()
      snode.parent = point
      snode.g = point.g + 1
      snode.orientation = 1
      snode.pos = S
      PQ.push(snode)
    if cells.cells.count(D) == 0 and path.cells.count(D) == 0 and frontier.cells.count(D) == 0:
      dnode = node()
      dnode.parent = point
      dnode.g = point.g + 1
      dnode.orientation = 2
      dnode.pos = D
      PQ.push(dnode)
    pass
    
    
    


def aStar(start,goal):
    print "``````````````````"
    global pq
    global cells
    global pubway
    global pubpath
    global path
    global frontier
    global onepath
    onepath = True
    clearpq = PQ()
    pq = clearpq
    
    
    
    nodelist = []
    frontier = GridCells()
    frontier.header.frame_id = 'map'
    frontier.cell_height = resolution
    frontier.cell_width = resolution
    
    realpath = GridCells()
    realpath.header.frame_id = 'map'
    realpath.cell_height = resolution
    realpath.cell_width = resolution
    
    path = GridCells()
    path.header.frame_id = 'map'
    path.cell_height = resolution
    path.cell_width = resolution
    
    wp = GridCells()
    wp.header.frame_id = 'map'
    wp.cell_height = resolution
    wp.cell_width = resolution
    
    s = point2Cell(start)
    s.z = 0
    g = point2Cell(goal)
    g.z = 0
    pq.goal = g
    
    wp.cells.append(g)
    realpath.cells.append(g)
    
    startnode = node()
    startnode.parent= None
    startnode.pos = s
    
    
    createNeighbor(startnode,pq)
    
    
    
    
    while((abs(startnode.pos.x-g.x)+abs(startnode.pos.y-g.y)) >= 0.5):
      startnode = pq.pop()[1]
      nodelist.append(startnode)
      path.cells.append(startnode.pos)
      createNeighbor(startnode,pq)
      #rospy.sleep(1)
      # print "enter while" ,abs(startnode.pos.x-g.x),abs(startnode.pos.y-g.y)
    #ort_previous = 0
      
    print "/////"
    while(startnode != None):
      realpath.cells.append(startnode.pos)
      ort_previous = startnode.orientation
      print ort_previous
      startnode = startnode.parent
      if startnode != None :
	  if(ort_previous != startnode.orientation) :
	      wp.cells.append(startnode.pos)
	      
	      # realpath.cells.remove(startnode.pos)
      
    
    print "exit while"
    wp.cells.reverse()
    onepath = False
    pubpath.publish(realpath)
    pubway.publish(wp)
      
    # create a new instance of the map
    


    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages

    # Publish points
    return path
  
def checkConnect(p, d):
    if((abs(p.x-d.x)+abs(p.y-d.y)) <= 0.6):
	return True
    return False
    
#publishes map to rviz using gridcells type

def publishCells(grid):
    global pub
    global pubpath
    global pubway
    global cells
    
    
    # resolution and offset of the map
    k=-1
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    
    for i in range(1,height+1): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (0.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
                
                
                left1 = Point()
                left1.x = point.x-resolution
                left1.y = point.y
                left1.z = 0
                cells.cells.append(left1)
                
                left2 = Point()
                left2.x = point.x-resolution-resolution
                left2.y = point.y
                left2.z = 0
                cells.cells.append(left2)
                
                left3 = Point()
                left3.x = point.x-resolution-resolution-resolution
                left3.y = point.y
                left3.z = 0
                cells.cells.append(left3)
                
                up1 = Point()
                up1.x = point.x
                up1.y = point.y-resolution
                up1.z = 0
                cells.cells.append(up1)
                
                up2 = Point()
                up2.x = point.x
                up2.y = point.y-resolution-resolution
                up2.z = 0
                cells.cells.append(up2)
                
                up3 = Point()
                up3.x = point.x
                up3.y = point.y-resolution-resolution-resolution
                up3.z = 0
                cells.cells.append(up3)
                
                right1 = Point()
                right1.x = point.x+resolution
                right1.y = point.y
                right1.z = 0
                cells.cells.append(right1)
                
                right2 = Point()
                right2.x = point.x+resolution+resolution
                right2.y = point.y
                right2.z = 0
                cells.cells.append(right2)
                
                right3 = Point()
                right3.x = point.x+resolution+resolution+resolution
                right3.y = point.y
                right3.z = 0
                cells.cells.append(right3)
                
                dw1 = Point()
                dw1.x = point.x
                dw1.y = point.y+resolution
                dw1.z = 0
                cells.cells.append(dw1)
                
                dw2 = Point()
                dw2.x = point.x
                dw2.y = point.y+resolution+resolution
                dw2.z = 0
                cells.cells.append(dw2)
                
                dw3 = Point()
                dw3.x = point.x
                dw3.y = point.y+resolution+resolution+resolution
                dw3.z = 0
                cells.cells.append(dw3)
                
                #leftup
                leftup1 = Point()
                leftup1.x = point.x-resolution
                leftup1.y = point.y-resolution
                leftup1.z = 0
                cells.cells.append(leftup1)
                
                leftup2 = Point()
                leftup2.x = point.x-resolution-resolution
                leftup2.y = point.y-resolution
                leftup2.z = 0
                cells.cells.append(leftup2)
                
                leftup3 = Point()
                leftup3.x = point.x-resolution
                leftup3.y = point.y-resolution-resolution
                leftup3.z = 0
                cells.cells.append(leftup3)
                
                leftup4 = Point()
                leftup4.x = point.x-resolution-resolution
                leftup4.y = point.y-resolution-resolution
                leftup4.z = 0
                cells.cells.append(leftup4)
                
                #leftdw
                leftdw1 = Point()
                leftdw1.x = point.x-resolution
                leftdw1.y = point.y+resolution
                leftdw1.z = 0
                cells.cells.append(leftdw1)
                
                leftdw2 = Point()
                leftdw2.x = point.x-resolution-resolution
                leftdw2.y = point.y+resolution
                leftdw2.z = 0
                cells.cells.append(leftdw2)
                
                leftup3 = Point()
                leftup3.x = point.x-resolution
                leftup3.y = point.y+resolution+resolution
                leftup3.z = 0
                cells.cells.append(leftup3)
                
                leftup4 = Point()
                leftup4.x = point.x-resolution-resolution
                leftup4.y = point.y+resolution+resolution
                leftup4.z = 0
                cells.cells.append(leftup4)
             
                #rightdw
                rightdw1 = Point()
                rightdw1.x = point.x+resolution
                rightdw1.y = point.y+resolution
                rightdw1.z = 0
                cells.cells.append(rightdw1)
                
                rightdw2 = Point()
                rightdw2.x = point.x+resolution+resolution
                rightdw2.y = point.y+resolution
                rightdw2.z = 0
                cells.cells.append(rightdw2)
                
                rightdw3 = Point()
                rightdw3.x = point.x+resolution
                rightdw3.y = point.y+resolution+resolution
                rightdw3.z = 0
                cells.cells.append(rightdw3)
                
                rightdw4 = Point()
                rightdw4.x = point.x+resolution+resolution
                rightdw4.y = point.y+resolution+resolution
                rightdw4.z = 0
                cells.cells.append(rightdw4)
                
                #rightup
                rightup1 = Point()
                rightup1.x = point.x+resolution
                rightup1.y = point.y-resolution
                rightup1.z = 0
                cells.cells.append(rightup1)
                
                rightup2 = Point()
                rightup2.x = point.x+resolution+resolution
                rightup2.y = point.y-resolution
                rightup2.z = 0
                cells.cells.append(rightup2)
                
                rightup3 = Point()
                rightup3.x = point.x+resolution
                rightup3.y = point.y-resolution-resolution
                rightup3.z = 0
                cells.cells.append(rightup3)
                
                rightup4 = Point()
                rightup4.x = point.x+resolution+resolution
                rightup4.y = point.y-resolution-resolution
                rightup4.z = 0
                cells.cells.append(rightup4)
    
    pub.publish(cells) 
    
def timerCallBack(event):
    global pose
    global trandone
    
    transdone = False
    
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint',rospy.Time(0))
    
    pose.position.x = position[0]
    pose.position.y = position[1]
  
    odomW = orientation
    q = [odomW[0],odomW[1],odomW[2],odomW[3]]
    roll,pitch,yaw = euler_from_quaternion(q)
    
    pose.orientation.z = yaw
    theta = math.degrees(yaw)
    transdone = True

#Main handler of the project
def run():
    global pub
    global goalFlag
    global startFlag
    global startPosX
    global startPosY
    global goalX
    global goalY
    global startP
    global goalP
    global pq
    global pubway
    global pubpath
    global onepath
    global pose
    goalFlag = False
    startFlag = False
    startP = Point()
    goalP = Point()
    onepath = True
    pose = Pose()
    
    
    rospy.init_node('lab3')
    #sub = rospy.Subscriber('', OccupancyGrid, mapCallBack)
    localsub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    
    goal_sub = rospy.Subscriber('move_base_simple/goal1', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('move_base_simple/init', Pose, readStart, queue_size=1) #change topic for best results
    
    # wait a second for publisher, subscribers, and TF
  
    rospy.sleep(1)
   

    
    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        
        if(goalFlag and startFlag and onepath):
	  
	  aStar(startP, goalP)
		






if __name__ == '__main__':
    
    try:
        run()
    except rospy.ROSInterruptException:
        pass

