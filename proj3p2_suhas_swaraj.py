import pygame
import numpy as np
import heapq
import time
import math

# initialising the map with all values as inf
map=np.full((600, 200), np.inf)

# The robot's parameters are defined as static/global variables
# The static variables' unit is centimeters

robot_radius = 22

wheel_radius= 6.6

wheel_dist = 28.7


# Take the input of step size, robot radius and clearance from the user
while True:
    rpm1 = int(input("Enter the wheel RPM 1 : "))
    rpm2 = int(input("Enter the wheel RPM 2 : "))
    clearance = int(input("Enter the obstacle clearance (in mm) : "))
    clearance=clearance*0.1
    break

# The offset from the obstacle is robot radius + clearance
offset = robot_radius + clearance

# Defining obstacle and offset space in the map using half planes and semi-algebraic models
for x in range(600):
    for y in range(200):
        x_bound = (x<=0+offset or x>=600-offset)
        y_bound = (y<=0+offset or y>=200-offset)
        circle = (((x-420)**2)+((y-120)**2)<=(60**2))
        circle_offset = (((x-420)**2)+((y-120)**2)<=((60+offset)**2))
        rec1_offset = (x>=150-offset and x<=175+offset) and (y>=100-offset and y<=200)
        rec1 = (x>=150 and x<=175) and (y>=100 and y<=200)
        rec2_offset = (x>=250-offset and x<=275+offset) and (y>=0 and y<=100+offset)
        rec2 = (x>=250 and x<=275) and (y>=0 and y<=100)
        
        # If the map region is within clearance space, the map array value is changed to -1
        if(rec1_offset or rec2_offset or circle_offset or x_bound or y_bound):
            map[x,y] = -1
                    
        # If the map region is within obstacle space, the map array value is changed to -2  
        if(rec1 or rec2 or circle):
            map[x,y] = -2
        
# Function to correctly define the angles    
def cor_angle(angle):
    if angle==360:
        angle=0
    if angle>360:
        angle=angle-360
    if angle<0:
        angle=360+angle
    return angle

# Function to round off the coordinate value (For checking duplicate nodes)
def approximate_coordinate(value):
    return round(value)

# Taking the initial node values from the user
while True:
    initial_x = int(input("Enter the initial x : "))
    initial_y = int(input("Enter the initial y : "))
    initial_theta = int(input("Enter the initial angle (theta in degrees) : "))
    initial_x=(initial_x+500)*0.1
    initial_y=(initial_y+1000)*0.1
    approx_initial_x = approximate_coordinate(initial_x)
    approx_initial_y = approximate_coordinate(initial_y)
    
    # If the given input is beyond the map dimensions, the input is asked for again 
    if initial_x>=600 or initial_x<0 or initial_y<0 or initial_y>=200:
        print("Please enter valid initial node value")
        continue
    
    # If the given input is in the obstacle space or clearance space of the map, the input is asked for again 
    if map[int(approx_initial_x),int(approx_initial_y)] < 0:
        print("Please enter valid initial node value")
        continue
    
    # If the given angle input is lesser than 0 degrees or greater than 360 degrees, input is asked again
    if initial_theta < 0 or initial_theta > 360:
        print("Please enter valid (positive) initial angle value between 0 and 360 degrees")
        continue
    break

# Taking the goal node values from the user
while True:
    goal_x = int(input("Enter the goal x : "))
    goal_y = int(input("Enter the goal y : "))
    goal_x = (goal_x + 500)*0.1
    goal_y = (goal_y + 1000)*0.1
    approx_goal_x = approximate_coordinate(goal_x)
    approx_goal_y = approximate_coordinate(goal_y)
    
    # If the given input is beyond the map dimensions, the input is asked for again 
    if goal_x>=600 or goal_x<0 or goal_y<0 or goal_y>=200:
        print("Please enter valid goal node value")
        continue
    
    # If the given input is in the obstacle space or clearance space of the map, the input is asked for again 
    if map[int(approx_initial_x),int(approx_initial_y)] < 0:
        print("Please enter valid goal node value")
        continue
    break            

# Variable to store the goal node coordinates
goal_node = (goal_x,goal_y)

# Variable representing the weight added to cost to go (Weighted A*)
weight = 2

# Calculation of cost to goal using Euclidean distance
def get_c2g(x,y):
    goal_x,goal_y = goal_node
    dist = math.sqrt(((goal_x-x)**2)+((goal_y-y)**2))
    return dist*weight

# Variable to store the cost to go for initial/starting node
initial_c2g = get_c2g(initial_x,initial_y)

# Defining the initial/starting node
initial_node = [(initial_x , initial_y , initial_theta , 0 , initial_c2g) , [(0,0,0)], []]

# Total Cost for initial/starting node
initial_total_cost = initial_c2g + 0

# Function to get the angular velocity from RPM
def get_angular_vel(rpm):
    return (2*math.pi*rpm)/60

# Function to get the new x, y and theta values after the robot executes an action
def get_x_y_theta(rpm_right,rpm_left,x,y,theta):
    # Extracting wheel angular velocities
    av_right=get_angular_vel(rpm_right)
    av_left=get_angular_vel(rpm_left)
    
    # Initially defining new x as x , new y as y and new theta as theta
    new_x=x
    new_y=y
    new_theta=theta    
    count=0   
    # Executing the loop 20 times at a time step of 0.05 to represent a time of 1 second (the action is executed for 1 second)
    while count<20:
        dx=(wheel_radius/2)*(av_right+av_left)*math.cos(math.radians(new_theta))*0.05
        dy=(wheel_radius/2)*(av_right+av_left)*math.sin(math.radians(new_theta))*0.05
        dtheta=(wheel_radius/wheel_dist)*(av_right-av_left)*0.05
        new_x=new_x+dx
        new_y=new_y+dy
        new_theta=cor_angle(new_theta+math.degrees(dtheta))
        count=count+1
    return new_x,new_y,new_theta

# Defining the 8 action sets based on wheel velocities / RPMs

def move1(node):
    v1=0 #right
    v2=rpm1 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history] 

    return new_node

def move2(node):
    v1=0 #right
    v2=rpm2 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history]

    return new_node

def move3(node):
    v1=rpm1 #right
    v2=0 # left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history]

    return new_node

def move4(node):
    v1=rpm2 #right
    v2=0 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history]

    return new_node

def move5(node):
    v1=rpm1 #right
    v2=rpm2 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history]

    return new_node

def move6(node):
    v1=rpm2 #right
    v2=rpm1 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history]

    return new_node

def move7(node):
    v1=rpm1 #right
    v2=rpm1 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history]

    return new_node

def move8(node):
    v1=rpm2 #right
    v2=rpm2 #left
    (x,y,theta,c2c,_)=node[0]
    path = node[1].copy() # Path for backtracking
    action_history=node[2].copy() # Recording all previous actions (wrt path) for giving as an input to turtlebot
    path.append((x, y, theta))
    action_history.append((v1,v2))
    new_x,new_y,new_theta=get_x_y_theta(v1,v2,x,y,theta)
    new_c2c = c2c + math.sqrt(((x-new_x)**2)+((y-new_y)**2))
    new_c2g = get_c2g(new_x,new_y)
    new_node=[(new_x,new_y,new_theta,new_c2c,new_c2g),path,action_history]

    return new_node

# Creating a open list to store unexplored nodes
open_list = []

# Converting open list to heapq
heapq.heapify(open_list)

# Creating a tracking array from the map array to keep track of explored/popped nodes
tracking_array_closed = map.copy()

# Creating a list to store explored nodes along with their parents (Backtracking)
closed_list=[]

# Pushing initial node with initial total cost into open list
# The open list sorts its elements according to the value of total cost
heapq.heappush(open_list, (initial_total_cost, initial_node))
searching = True
possible_actions = [move1,move2,move3,move4,move5,move6,move7,move8]

#A Star Algorithm Implementation
while searching:
    # pops the node with the least total cost
    (total_cost,node) = heapq.heappop(open_list)
    # Extracting Values from the popped node
    (x,y,theta,c2c,c2g)=node[0]
    # Changing the value of the tracking array based on the popped node to indicate that it has been explored
    tracking_array_closed[int((approximate_coordinate(x))),int((approximate_coordinate(y)))] = 2
    #Checking if the goal node is within the distance threshold of the present node
    if c2g <= 2:
        print("Solution reached")
        searching = False
        final_node = node
    else:
        #Perform actions, get new node, c2g, c2c and total cost
        for move_function in possible_actions:
            new_node = move_function(node)
            (new_x, new_y, new_theta,new_c2c,new_c2g) = new_node[0]
            new_total_cost = new_c2c + new_c2g
            # Creating a variable to check if there is a similar node in open list with higher total cost
            test = (approximate_coordinate(new_x), approximate_coordinate(new_y))
            # If the new node is beyond the map dimensions (due to step size), skip the node
            if (int((approximate_coordinate(new_x))))>=int(600) or (int((approximate_coordinate(new_y))))>=int(200):
                continue
            # if the new node is in obstacle or offset space, skip the node
            if map[int((approximate_coordinate(new_x))),int((approximate_coordinate(new_y)))] < 0:
                continue
            # if the new node is in closed, skip the node. This is checked using a 2d array
            if tracking_array_closed[int(approximate_coordinate((new_x))),int((approximate_coordinate(new_y)))]==2:
                continue
            # The tracking_node and closed list are used for plotting the explored nodes
            tracking_node=[new_x,new_y,x,y]
            closed_list.append(tracking_node)
            # if the node is in open list, check if the new c2c < old c2c, if so, replace the node. 
            if map[int(approximate_coordinate((new_x))),int((approximate_coordinate(new_y)))]==1:
                for i, (_, open_node) in enumerate(open_list):
                    (open_x,open_y,open_theta,open_c2c,_) = open_node[0]
                    open_test = (approximate_coordinate(open_x),approximate_coordinate(open_y))
                    if open_test==test:
                        if new_c2c < open_c2c:
                            open_list[i] = (new_total_cost, new_node)
                            break
            else:
                # Adding new modes to the open list keeping a track of the exploration
                map[int((approximate_coordinate(new_x))),int((approximate_coordinate(new_y)))] = 1
                heapq.heappush(open_list, (new_total_cost, new_node))
        # If the oprn list is empty at any point, no solution is found
        if not open_list:
            print("No solution found")
            break
                
# Animating using pygame
# Obstacles are represented in red
# Clearance/offset is represented in yellow
# Free space is represented in white
# Node exploration is represented in pink
# Optimal path is represented in black

pygame.init()
screen = pygame.display.set_mode((600, 200))
while True:
    # Indexing for skipping the first value, a dummy value which we added earlier
    n=0
    m=0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    for y in range(map.shape[1]):
        for x in range(map.shape[0]):
            if map[x, y] >= 0:
                pygame.draw.rect(screen, (255, 255, 255), (x , 200-y , 1, 1))
            elif map[x, y] == -2:
                pygame.draw.rect(screen, (255, 0, 0), (x , 200-y , 1, 1))
            elif map[x, y] == -1:
                pygame.draw.rect(screen, (255, 255, 0), (x, 200-y , 1, 1))
    
    pygame.draw.rect(screen, (0, 0, 0), (goal_node[0] , 200-goal_node[1] , 1, 1))
    pygame.draw.rect(screen, (0, 0, 0), (initial_node[0][0] , 200-initial_node[0][1] , 1, 1))
    
    for i in closed_list:
        if n==0:
            n=n+1
            continue
        (x_current,y_current,x_previous,y_previous) = i
        pygame.draw.line(screen, (255,150,140), (x_previous, 200-y_previous), (x_current,200-y_current),1)
        pygame.display.update()
    path=final_node[1] 
    for i in range(len(path)-1):
        if m==0:
            m=m+1
            continue
        (x1,y1,_)=path[i]
        
        (x2,y2,_)=path[i+1]
        pygame.draw.line(screen, (0,0,0), (x1,200-y1),(x2,200-y2),2)
    
    x3,y3,_=path[-1]
    x4, y4, _, _, _ = final_node[0]
    pygame.draw.line(screen, (0,0,0), (x3,200-y3),(x4,200-y4),2)
    pygame.display.update()
    time.sleep(10)
pygame.quit()
