#!/usr/bin/env python

import rospy, tf, numpy, math, heapq
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent


class node(object):
    def __init__(self):
	
	self.parent = None
	self.pos= Point()
	self.orientation = 0
	self.g = 0
class PQ(object):
    global resolution
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
	heapq.heappush(self.dist, ((heuristic(node.pos,self.goal)+node.g),node))
	#print heuristic(node.pos,self.goal)+node.g
	self.cells.cells.append(node.pos)
	frontier.cells.append(node.pos)
    def pop(self):
	global frontier
	self.cells.cells.remove(self.dist[0][1].pos)
	
	frontier.cells.remove(self.dist[0][1].pos)
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
    
    print offsetX, offsetY
    print "updateMap"
    

def readGoal(goal):
    global goalX
    global goalY
    global goalP
    global goalFlag
    global onepath
    
    goalP = Point()
    goalFlag = True
    
    goalX= goal.position.x
    goalY= goal.position.y
    
    goalP.x = goalX
    goalP.y = goalY
    goalP.z = 0
    
    # print "readGoal"
    # print goal.pose
    
    goalFlag = True
    onepath = True
    
def readGoal4test(goal):
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
    # return abs(current.x-goal.x)+abs(current.y-goal.y)
    return math.sqrt(math.pow((current.x-goal.x),2)+math.pow((current.y-goal.y),2))

def point2Cell(point):
    global resolution
    global height
    global width
    pointreturn = Point()
    pointreturn.x = int((point.x-offsetX - (0.5 * resolution))/resolution)
    pointreturn.y = int((point.y-offsetY + (0.5 * resolution))/resolution)    
    
    pointreturn.z = (pointreturn.y-1)*width+pointreturn.x
  
    point=Point()
    pointreturn.x=(pointreturn.x*resolution)+offsetX + (0.5 * resolution) # added secondary offset 
    pointreturn.y=(pointreturn.y*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?

    

    return pointreturn


def point2grid(point):
    global resolution
    global height
    global width
    j = int((point.x-0.5*resolution-offsetX)/resolution)
    i = int((point.y+0.5*resolution-offsetY)/resolution)
    return (i-1)*width+j-1

def createNeighbor(point,PQ):
    global path
    global frontier
    global cells
    global nodelist
    global resolution
    global height
    global width
    W = Point(point.pos.x,point.pos.y+resolution,point.pos.z)
    A = Point(point.pos.x-resolution,point.pos.y,point.pos.z)
    D = Point(point.pos.x+resolution,point.pos.y,point.pos.z)
    S = Point(point.pos.x,point.pos.y-resolution,point.pos.z)
    
    kw = point2grid(W)
    ka = point2grid(A)
    ks = point2grid(S)
    kd = point2grid(D)
    #print kw
    #print len(cells.cells), len(path.cells), len(frontier.cells)
    if aMap[kw] < 70 and nodelist[point2Cell(W).z] == 0 :
      nodelist[point2Cell(W).z]=2
      wnode = node()
      wnode.parent = point
      wnode.g = point.g + resolution
      wnode.orientation = 1
      wnode.pos = W
      PQ.push(wnode)
    #print ka
    if aMap[ka] < 70 and nodelist[point2Cell(A).z] == 0:
      nodelist[point2Cell(A).z]=2
      anode = node()
      anode.parent = point
      anode.pos = A
      anode.g = point.g + resolution
      anode.orientation = 2
      PQ.push(anode)
    #print ks
    if aMap[ks] < 70 and nodelist[point2Cell(S).z] == 0:
      nodelist[point2Cell(S).z]=2
      snode = node()
      snode.parent = point
      snode.g = point.g + resolution
      snode.orientation = 1
      snode.pos = S
      PQ.push(snode)
    #print kd
    if aMap[kd] < 70 and nodelist[point2Cell(D).z] == 0:
      nodelist[point2Cell(D).z]=2
      dnode = node()
      dnode.parent = point
      dnode.g = point.g + resolution
      dnode.orientation = 2
      dnode.pos = D
      PQ.push(dnode)
    pubfront.publish(frontier)
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
    global nodelist
    global resolution
    global height
    global width
    onepath = True
    clearpq = PQ()
    pq = clearpq
    
    a = int((startP.y+0.5*resolution-offsetY)/resolution)
    b = int((startP.x-0.5*resolution-offsetX)/resolution)
    k = (a-1)*width+b-1
    if(aMap[k] == 100):
      for i in range(a-2,a+3):
	for j in range(b-2,b+3):
	  k = (i-1)*width+j-1
	  aMap[k] = 0
    
    print "starting A*"
    
    nodelist = [0] *len(aMap)
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
    
    
    
    print "expanding frontiers"
    while((abs(startnode.pos.x-g.x)+abs(startnode.pos.y-g.y)) >= 0.5):
      startnode = pq.pop()[1]
      nodelist[point2Cell(startnode.pos).z]=1
      path.cells.append(startnode.pos)
      
      createNeighbor(startnode,pq)
      
      #pubpath.publish(path)
      #rospy.sleep(1)
      # print "enter while" ,abs(startnode.pos.x-g.x),abs(startnode.pos.y-g.y)
    #ort_previous = 0
      
    print "looking for waypoints"
    while(startnode != None):
      realpath.cells.append(startnode.pos)
      ort_previous = startnode.orientation
      #print ort_previous
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
    global aMap
    global resolution
    global height
    global width
    
    aMap = [0]*500000
    
    # resolution and offset of the map
    k=-1
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    print height, width
    for i in range(1,height+1):
      for j in range(1,width):
	k = (i-1)*(width)+j-1
	#print k
	if(k<(height*width)):
	  
	  if(grid[k] > 99):
	  
	    for m in range(i-4,i+4):
	      for n in range(j-4,j+4):
		if (math.sqrt(math.pow((m-i),2)+math.pow((n-j),2))<5):
		  k = (m-1)*width+n-1
		  #print k
		
		  aMap[k] = 100
		  point=Point()
		  point.x=(n*resolution)+offsetX + (0.5 * resolution) # added secondary offset 
		  point.y=(m*resolution)+offsetY - (0.5 * resolution) # added secondary offset ... Magic ?
		  point.z=0
		  cells.cells.append(point)
		  
    a = int((startP.y+0.5*resolution-offsetY)/resolution)
    b = int((startP.x-0.5*resolution-offsetX)/resolution)
    k = (a-1)*width+b-1
    if(aMap[k] == 100):
      for i in range(a-2,a+3):
	for j in range(b-2,b+3):
	  k = (i-1)*width+j-1
	  aMap[k] = 0
    
    print len(cells.cells)
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
    global pubfront
    global onepath
    global pose
    global mapData
    goalFlag = False
    startFlag = False
    startP = Point()
    goalP = Point()
    onepath = True
    pose = Pose()
    
    
    rospy.init_node('lab3')
    #sub = rospy.Subscriber('', OccupancyGrid, mapCallBack)
    localsub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    costsub = rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, mapCallBack)
    #move_base/global_costmap/costmap
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    pubfront = rospy.Publisher("/frontier", GridCells, queue_size = 1)
    
    goal_sub = rospy.Subscriber('move_base_simple/goal1', Pose, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('move_base_simple/init', Pose, readStart, queue_size=1) #change topic for best results
    #goal_sub = rospy.Subscriber('move_base_simple/goal3', PoseStamped, readGoal4test, queue_size=1) #change topic for best results
    # wait a second for publisher, subscribers, and TF
  
    rospy.sleep(1)
   

    
    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        
        if(goalFlag and startFlag and onepath):
	  print "in while loop and starting a*"
	  aStar(startP, goalP)
		






if __name__ == '__main__':
    
    try:
        run()
    except rospy.ROSInterruptException:
        pass

