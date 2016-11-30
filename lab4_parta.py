#!/usr/bin/env python

import rospy, tf, math
from nav_msgs.msg import GridCells
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion


# Add additional imports for each of the message types used

def beAlertToCheck(event):
    
    pubinit.publish(pose)
    print "checkpoint"
    print "spin and check"
    
    


# NavToPose
def navToPose(goal):
    global timeToCheck
    timeToCheck = False
    valid = True
    
    
    buff = goal.cells[1]
    goal.cells.remove(buff)
    navToPoint(buff)
    pubinit.publish(pose)
    if(goal.cells.__len__() == 0):
	valid = False
	
	  #rotate(179)
	  #rotate(179)
	
    
    timeToCheck = True
      
#drive to a goal subscribed as /move_base_simple/goal
def navToPoint(goal):
    if(goal != -1):
	while(not transdone):
	    rospy.sleep(0.15)
	goalx = goal.x
	goaly = goal.y
	goalz = 0
	#goalz = goal.pose.orientation.z
	currentx = pose.position.x
	currenty = pose.position.y
	currentz = pose.orientation.z
	firstAngle = math.atan2((goaly-currenty),(goalx-currentx))
	
	while ((abs(math.degrees(firstAngle-currentz))) >= 1):
	    rospy.sleep(0.15)
	    
	    if(math.degrees(firstAngle-currentz)>0):
		    publishTwist(0,0.5)
		    
	    else:
		    publishTwist(0,-0.5)
		    
	    currentz = pose.orientation.z

	
	
	dis = math.sqrt(math.pow((goalx-currentx),2)+math.pow((goaly-currenty),2))
	if (dis<1):
	    driveStraight(0.3,dis)
	else :
	    driveStraight(0.3,1)
	#while ((abs(math.degrees(goalz-currentz))) >= 1):
	#    rospy.sleep(0.15)
	#    
	#    if(math.degrees(goalz-currentz)>0):
	#	    publishTwist(0,0.5)
	#	    
	#    else:
	#	    publishTwist(0,-0.5)
	#	    
	#    currentz = pose.orientation.z
	
	print "done"
	return True
    return False


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    hitBump = False
    driveStraight(0.1,0.6)
    rotate(-90)
    driveStraight(0.1,0.45)
    rotate(135)


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
  
    while(not transdone):
      rospy.sleep(0.15)
    
    lin_vel = (u1+u2)/2
    ang_vel = (u2-u1)*(1/0.35)

    twist_msg = Twist();
    stop_msg = Twist();

    twist_msg.linear.x = lin_vel
    twist_msg.angular.z = ang_vel
    stop_msg.linear.x = 0;
    stop_msg.angular.z = 0

    now = rospy.Time.now().secs 
    while(rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
    	pub.publish(twist_msg)
    pub.publish(stop_msg)



#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
  
    while(not transdone):
	rospy.sleep(0.15)

    
    initialX= pose.position.x
    initialY = pose.position.y

    atTarget = False
    
    while(not atTarget and not rospy.is_shutdown() and not hitBump):
    	currentX = pose.position.x
    	currentY = pose.position.y
    	atTarget = False
	currentDistance = math.sqrt(math.pow((currentX-initialX),2)+math.pow((currentY-initialY),2))
    	if(currentDistance >= distance):
    		atTarget = True
    		publishTwist(0,0)
    		print "stop!"
    	elif(currentDistance >= (distance-0.6) and currentDistance < distance):
		dis = distance-currentDistance
		de = dis/0.2*speed+0.05
		publishTwist(de,0)
		
	elif(currentDistance <= 0.6 and currentDistance>0.2):
		ac = currentDistance/0.2*speed+0.1
		publishTwist(ac,0)
		
    	else:
    		publishTwist(speed,0)
    		rospy.sleep(0.15)   
    
    
#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    print "rotate"
    global transdone
    while(not transdone):
	rospy.sleep(0.15)
    
    currentZ = math.degrees(pose.orientation.z)
    
   
   
    if(angle > 180 or angle < -180):
    	print"angle is too large or small"
    vel = Twist();
    done = True
    goal = currentZ+angle
    if abs(goal)> 180:
	if goal >0:
		goal= goal-360
	else:
		goal=goal+360
    error = err(goal,math.degrees(pose.orientation.z),angle)
    sign = isPositive(currentZ)
    
    while ((abs(error) >= 2)):
	rospy.sleep(0.15)
	shabi = math.degrees(pose.orientation.z)
	dashabi = error
	if(error>0):
		publishTwist(0,1)
		
	else:
		publishTwist(0,-1)
		  
	error = err(goal,math.degrees(pose.orientation.z),angle)

    done = True
    publishTwist(0,0)
    vel.angular.z = 0.0
    pub.publish(vel)
    print "done rotate"
    rospy.sleep(0.15)
def err(goal,cur,sign):
    if(goal>0 and cur>0) or (goal<0 and cur<0):
      error = goal - cur
    else:
	error = 360-abs(goal)-abs(cur)
        if sign < 0:
	  error = -error
    
    return error

def isPositive(num):
    if(num>=0):
      return True
    else:
      return False


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    print "drivearc"
    global transdone
    while(not transdone):
      rospy.sleep(0.15)
      
    r = 0.235
    v1 = (radius-r/2)/radius*speed
    v2 = (radius+r/2)/radius*speed
    t1 = float(angle)/360*radius*2*math.pi/speed
    
    if(v1>1 or v1<-1 or v2>1 or v2<-1):
      print"Move too fast"
      return
    
    spinWheels(v1,v2,t1)
    
    


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        hitBump = True        
        print"Bumper pressed!"
        executeTrajectory()


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    
    global xPosition
    global yPosition
    global theta
    global transdone
    global hitBump
    
    transdone = False
    hitBump = False
    
    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
    (position, orientation) = odom_list.lookupTransform('map','base_footprint',rospy.Time(0))
    pose.position.x = position[0]
    pose.position.y = position[1]
    
    xPosition=position[0]
    yPosition=position[1]
    
    odomW = orientation
    q = [odomW[0],odomW[1],odomW[2],odomW[3]]
    roll,pitch,yaw = euler_from_quaternion(q)
    
    pose.orientation.z = yaw
    theta = math.degrees(yaw)
    
    transdone = True
    

def publishTwist(linearVelocity,angularVelocity):
    global pub 
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)
   



# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    
    global pub
    global pose
    global odom_tf
    global odom_list    
    global transdone
    
        
    pose = Pose()
    transdone = False
    
    
    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None,queue_size=10) # Publisher for commanding robot motion
    pubinit = rospy.Publisher('move_base_simple/init', Pose, queue_size = 1)
    #bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('/waypoints',GridCells,navToPose,queue_size=1)
   
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))



    print "Starting Lab 2"

    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(0.1), timerCallback)
    
    
    rospy.sleep(5)
    pubinit.publish(pose)
    odom_list = tf.TransformListener()
    
    # Make the robot do stuff...
    
    #straight
    #driveStraight(0.5,1)
    
    #rotate
    
    
    
    #nav
    while(1):
      rospy.sleep(.15)
    
    print "Lab 2 complete!"

