# c:\users\bastien\appdata\local\programs\python\python39\python
import sys,os
sys.path.append('C:\Program Files\Webots\lib\controller\python39')
import math
import time;
from controller import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import random

from matplotlib import collections  as mc
import random
from scipy.spatial import cKDTree as KDTree        

# Define the robot's operation class: forward() backward() left() right() stop()
class Operation:
    def __init__(self):
        self.speed = 5

    def forward(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(self.speed)
        motor_right.setVelocity(self.speed)
    
    def back(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(-self.speed)
        motor_right.setVelocity(-self.speed)

    def rotate_L(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(-self.speed)
        motor_right.setVelocity(self.speed)

    def rotate_R(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(self.speed)
        motor_right.setVelocity(-self.speed)

    def stop(self):
        self.speed = 0
        motor_left.setVelocity(self.speed)
        motor_right.setVelocity(self.speed)
        
        
# Robot right rotation at a specified angle (under angular system)   
def Robot_R(robot, node, motor_left, motor_right, operation, degree, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    rotation = degree/90*80 # Set the number of operations required to rotate the specified angle
    counter = 0 # Init counter
    while (robot.step(timestep) != -1):
        if(counter>rotation): # Jump out when to the correct angle
            break
        counter += 1
        operation.rotate_R()
        deplacement_left, deplacement_right = calculate_wheel_deplacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100) # Becauuse node.getPosition will return in meter 
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real

# Robot left rotation at a specified angle (under angular system)   
def Robot_L(robot, node, motor_left, motor_right, operation, degree, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    rotation = degree/90*80
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>rotation):
            break
        counter += 1
        operation.rotate_L()
        deplacement_left, deplacement_right = calculate_wheel_deplacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100)
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real

# Robot backward at a specified distance (the multiplier of the lattice)   
def Robot_B(robot, node, motor_left, motor_right, operation, square, pos_x, pos_y, thet, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    step = square*250 # Set the number of operations required to rotate the specified angle
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>step):
            break
        counter += 1
        operation.back()
        deplacement_left, deplacement_right = calculate_wheel_deplacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100)
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real

# Robot forkward at a specified distance (the multiplier of the lattice)   
def Robot_F(robot, node, motor_left, motor_right, operation, square, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    step = square*250 
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>step):
            break
        counter += 1
        operation.forward()
        deplacement_left, deplacement_right = calculate_wheel_deplacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100)
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real


# Using the keyboard to control robot movement      
def keyBoard_Controller(keyboard, node, lidar, robot, operation, motor_left, motor_right, pos_x, pos_y, theta, plan_xy):

    interactive = False
    fig = plt.figure()

    calibration_x = pos_x
    calibration_y = pos_y
    # Record history of position 
    list_pos_x = []
    list_pos_y = []
    list_pos_x_real = []
    list_pos_y_real = []
    list_scatter_x = []
    list_scatter_y = []
    list_scatter_pt_x = []
    list_scatter_pt_y = []
    list_scatter_pt_x_est = []
    list_scatter_pt_y_est = []
    
    while (robot.step(timestep) != -1): #Appel d'une 脙漏tape de simulation
        key=keyboard.getKey()
        if (key==keyboard.UP) :
            operation.forward()
        elif (key==keyboard.DOWN) :
            operation.back()
        elif (key==keyboard.LEFT) :
             operation.rotate_L()
        elif (key==keyboard.RIGHT) :
             operation.rotate_R()
        elif (key==ord('S')) :
            break
        else:
            operation.stop()
            
        # real position
        xyz =  node.getPosition()
        rotation = node.getOrientation()
        
        # simulated position
        deplacement_left, deplacement_right = calculate_wheel_deplacement(motor_left, motor_right)        
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        xyz_est = [ pos_y/100, 0.0, pos_x/100]
        rotation_est = theta
            
        # Get lidar data
        point_cloud = lidar.getRangeImage()
        # print(point_cloud)
        plan_x, plan_y, plan_pt_x, plan_pt_y, plan_pt_x_est, plan_pt_y_est = polar_2_cart(point_cloud, xyz, rotation, xyz_est, rotation_est)
        
        list_scatter_x.append(plan_x)
        list_scatter_y.append(plan_y)
        plan_xy = [list_scatter_x, list_scatter_y]
        
        list_scatter_pt_x.append(plan_pt_x)
        list_scatter_pt_y.append(plan_pt_y)
        plan_pt_xy = [list_scatter_pt_x, list_scatter_pt_y]
        
        list_scatter_pt_x_est.append(plan_pt_x_est)
        list_scatter_pt_y_est.append(plan_pt_y_est)
        plan_pt_xy_est = [list_scatter_pt_x_est, list_scatter_pt_y_est]
        
        # Add position to history
        pos_x_real, pos_y_real = node.getPosition()[2], node.getPosition()[0]
        list_pos_x_real.append(pos_x_real * 100)
        list_pos_y_real.append(pos_y_real * 100)

        
        # If detect specialized signal from keyboard then print the position state
        if key==ord('Z') or key==ord('S') or key==ord('Q') or key==ord('D'):
            # print(f'Deplacement for Left Wheel: {deplacement_left}            Deplacement for Right Wheel: {deplacement_right} ')
            print(f'position state : [x={round(pos_x,1)} , y={round(pos_y,1)}]')
            # print(f'Deplacement for left {round(deplacement_distance[0], 1)} cm ; right {round(deplacement_distance[1], 1)} cm')
        
        # Monitor of the position in real time
        if interactive:
            update_plot(fig, interactive, list_pos_x_real, list_pos_y_real, plan_xy, plan_pt_xy, plan_pt_xy_est)
        else:
            pass
    update_plot(fig, interactive, list_pos_x_real, list_pos_y_real, plan_xy, plan_pt_xy, plan_pt_xy_est)


# Construct position around by cloud    
def polar_2_cart(point_cloud, xyz, rotation, xyz_est, rotation_est):

    list_x, list_y, list_pt_x, list_pt_y, list_pt_x_est, list_pt_y_est = [], [], [], [], [], []
    angle = 0
    for i in point_cloud:
        #xy = [i*math.cos(angle)*100, 0, i*math.sin(angle)*100]
        xy = [i*math.sin(angle)*100, 0, i*math.cos(angle)*100]
        list_x.append(xy[2])
        list_y.append(-xy[0])
        
        pt = multmatr(rotation, xy, xyz)
        list_pt_x.append(pt[2])
        list_pt_y.append(-pt[0])

        R = np.array([[np.cos(rotation_est), -np.sin(rotation_est)], [np.sin(rotation_est), np.cos(rotation_est)]])
        T = np.array([-xyz_est[0]*100, xyz_est[2]*100]) 
        X = np.array([xy[0], xy[2]])
        
        pt = np.dot(R, X) + T
        #pt_est = multmatr(rotation_est, xy, xyz_est)
        list_pt_x_est.append(pt[1])
        list_pt_y_est.append(-pt[0])
        
        angle += 2*math.pi / lidar.getHorizontalResolution()
    
    return list_x, list_y, list_pt_x, list_pt_y, list_pt_x_est, list_pt_y_est
   

# RX+T
def multmatr(R,X,T):
    res = []
    res.append( R[0] * X[0] + R[3] * X[1] + R[6] * X[2] - T[0]*100)
    res.append( R[1] * X[0] + R[4] * X[1] + R[7] * X[2] + T[1]*100)
    res.append( R[2] * X[0] + R[5] * X[1] + R[8] * X[2] + T[2]*100)
  
    return res
    
    
    
def update_plot(fig, interactive, list_pos_x_real, list_pos_y_real, plan_xy, plan_pt_xy, plan_pt_xy_est):
    
    if interactive:
        # Interactive mode
        plt.ion()
    
    """
    sample_plan_xy = [[],[]]
    sample_plan_xy_est = [[],[]]
    if len(plan_pt_xy[0])> 200:
        sample_size = 200
        sample_indices = random.sample(range(len(plan_pt_xy[0])), sample_size)
        sample_plan_xy[0] = [plan_pt_xy[0][i] for i in sample_indices]
        sample_plan_xy[1] = [plan_pt_xy[1][i] for i in sample_indices]
        
        sample_indices = random.sample(range(len(plan_pt_xy_est[0])), sample_size)
        sample_plan_xy_est[0] = [plan_pt_xy_est[0][i] for i in sample_indices]
        sample_plan_xy_est[1] = [plan_pt_xy_est[1][i] for i in sample_indices]
        
    else:
        sample_plan_xy = plan_xy
    """
 
    sample_plan_xy = plan_pt_xy
    sample_plan_xy_est = plan_pt_xy_est
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(sample_plan_xy[0], sample_plan_xy[1], s=1,  label='Real Obs')
    ax.scatter(sample_plan_xy_est[0], sample_plan_xy_est[1], s=1,  label='Estimated Obs')

    # Plot simulated and real robots
    ax.plot(list_pos_x_real, list_pos_y_real, label='Real Robot')

    # Pre-set internal walls
    ax.plot([0, -50], [0, 0], color='black', linestyle='-', linewidth=5, label='Internal Wall')
    ax.plot([0, 0], [50, -50], color='black', linestyle='-', linewidth=5)
    ax.plot([0, 50], [50, 50], color='black', linestyle='-', linewidth=5)
    
    # Pre-set other walls
    ax.plot([-25, 75], [75, 75], color='black', linestyle='-', linewidth=2)
    ax.plot([75,75], [75, 25], color='black', linestyle='-', linewidth=2)
    ax.plot([75, 25], [25, 25], color='black', linestyle='-', linewidth=2)
    ax.plot([25, 25], [25, -75], color='black', linestyle='-', linewidth=2)
    ax.plot([25, -25], [-75, -75], color='black', linestyle='-', linewidth=2)
    ax.plot([-25, -25], [-75, -25], color='black', linestyle='-', linewidth=2)
    ax.plot([-25, -75], [-25, -25], color='black', linestyle='-', linewidth=2)
    ax.plot([-75, -75], [25, -25], color='black', linestyle='-', linewidth=2)
    ax.plot([-25, -75], [25, 25], color='black', linestyle='-', linewidth=2)
    ax.plot([-25, -25], [25, 75], color='black', linestyle='-', linewidth=2)
    
    # Set labels
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.legend()
    
    # Set view angle to top-down (90 degrees)
    ax.view_init(90, 180)
    
    ax.zaxis.set_ticks([])

    plt.draw()
    if interactive:
        plt.pause(0.001)
        plt.clf()
    else:
        plt.show()
    
 
# Calculate deplacement for dt
def calculate_wheel_deplacement(motor_left, motor_right):

    # deplacement = v*dt = w*r*dt
    deplacement_left = motor_left.getVelocity() * WHEEL_RADIUS * dt
    deplacement_right = motor_right.getVelocity() * WHEEL_RADIUS * dt

    return deplacement_left, deplacement_right
    

# Update position as the formula
def update_robot_position(displacement_left, deplacement_right, pos_x, pos_y, theta):

    delta_s = (deplacement_right + displacement_left) / 2
    delta_theta = (deplacement_right - displacement_left) / wheel_base

    # Update the robot's orientation
    theta += delta_theta

    # Update the robot's position
    pos_x += delta_s * math.cos(theta + delta_theta / 2)
    pos_y += delta_s * math.sin(theta + delta_theta / 2)
    theta = theta % (2 * math.pi)  # Keep theta within [0, 2脧鈧琞
    return pos_x, pos_y, theta
    
def indxtMean(index,arrays):
    indxSum = np.array([0.0, 0.0 ,0.0])
    for i in range(np.size(index,0)):
        indxSum = np.add(indxSum, np.array(arrays[index[i]]), out = indxSum ,casting = 'unsafe')
    return indxSum/np.size(index,0)
    
def indxtfixed(index,arrays):
    T = []
    for i in index:
        T.append(arrays[i])
    return np.asanyarray(T)
    
def ICPSVD(fixedX,fixedY,movingX,movingY):
    reqR = np.identity(3)
    reqT = [0.0, 0.0, 0.0]
    fixedt = []
    movingt = []
    for i in range(len(fixedX)):
        fixedt.append([fixedX[i], fixedY[i], 0])
    for i in range(len(movingX)):
        movingt.append([movingX[i], movingY[i], 0]) 
    moving = np.asarray(movingt)
    fixed = np.asarray(fixedt)

    n = np.size(moving,0)
    TREE = KDTree(fixed)
    for i in range(10):
        distance, index = TREE.query(moving)
        err = np.mean(distance**2) 
        com = np.mean(moving,0)
        cof = indxtMean(index,fixed)
        W = np.dot(np.transpose(moving),indxtfixed(index,fixed)) - n*np.outer(com,cof) 
        U , _ , V = np.linalg.svd(W, full_matrices = False) 
        tempR = np.dot(V.T,U.T)
        tempT = cof - np.dot(tempR,com)
 
        moving = (tempR.dot(moving.T)).T
        moving = np.add(moving,tempT) 
        reqR=np.dot(tempR,reqR)
        reqT = np.add(np.dot(tempR,reqT),tempT)
        
# 灏嗗娈电鏁ｅ寲涓虹偣浜�
def discretize_wall_segments(step_size=1):

    segments = [
    [(0, 0), (-50, 0)],  
    [(0, 0), (0, 50)],
    [(0, 50), (50, 50)],
    
    [(-25, 75), (75, 75)],
    [(75, 75), (75, 25)],
    [(75, 25), (25, 25)],
    [(25, 25), (25, -75)],
    [(25, -75), (-25, -75)],
    [(-25, -75), (-25, -25)],
    [(-25, -25), (-75, -25)],
    [(-75, -25), (-75, 25)],
    [(-75, 25), (-25, 25)],
    [(-25, 25), (-25, 75)]
    ]

    points_x, points_y = [], []
    for segment in segments:
        start, end = segment
        # 璁＄畻娈电殑闀垮害鍜屾柟鍚�
        length = np.linalg.norm(np.array(end) - np.array(start))
        direction = (np.array(end) - np.array(start)) / length
        # 灏嗘鍒嗚В涓烘闀夸负1cm鐨勭偣
        for i in range(int(length / step_size) + 1):
            point = np.array(start) + i * step_size * direction
            points_x.append(point[0])
            points_y.append(point[1])
    plt.scatter(points_x, points_y)

    plt.title('Discretized Wall Segments')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.show()
    return points_x, points_y
         
if __name__=="__main__":
    # size of the square 0.25cm*0.25cm
    # Do initialization and instantiation
    robot = Supervisor()
    node = robot.getFromDef("Thymio")

    motor_left = robot.getDevice("motor.left");
    motor_right = robot.getDevice("motor.right");
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    timestep = int(robot.getBasicTimeStep())
    
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()
    
    operation = Operation()
    keyboard=Keyboard()
    keyboard.enable(timestep)
    
    # Pre-set constant
    WHEEL_RADIUS = 2.105 # in cm
    TREAD = 10.8 # in cm
    wheel_base = TREAD 
    dt = timestep/1000
    pos_x = node.getPosition()[2]*100  # init x position  
    pos_y = node.getPosition()[0]*100  # init y position  
    theta = 0  # orientation in radians
    plan_xy = [0, 0]
    
    print(f'Initial Position State : [x={round(pos_x,1)} , y={round(pos_y,1)}, theta={round(theta,1)}]')
    
    # Passive-control by Keyboard
    print("keyboard controller activated [####################] 100%")
    #keyBoard_Controller(keyboard, node, lidar, robot, operation, motor_left, motor_right, pos_x, pos_y, theta, plan_xy)
    print(">> Exit keyboard control")
    discretize_wall_segments()
    
    
   
    
    
    
    
   
  
        
    

   
    
    


        
        
        
        