# c:\users\bastien\appdata\local\programs\python\python39\python
import sys,os
sys.path.append('C:\Program Files\Webots\lib\controller\python39')
import math
import time;
from controller import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

from matplotlib import collections  as mc
import random
from scipy.spatial import cKDTree as KDTree        

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
        
def Robot_R(robot, node, motor_left, motor_right, operation, degree, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    rotation = degree/90*80
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>rotation):
            break
        counter += 1
        operation.rotate_R()
        deplacement_left, deplacement_right = calculate_wheel_displacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100)
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real

def Robot_L(robot, node, motor_left, motor_right, operation, degree, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    rotation = degree/90*80
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>rotation):
            break
        counter += 1
        operation.rotate_L()
        deplacement_left, deplacement_right = calculate_wheel_displacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100)
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real

def Robot_B(robot, node, motor_left, motor_right, operation, square, pos_x, pos_y, thet, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    step = square*250
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>step):
            break
        counter += 1
        operation.back()
        deplacement_left, deplacement_right = calculate_wheel_displacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100)
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real

def Robot_F(robot, node, motor_left, motor_right, operation, square, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    step = square*250
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>step):
            break
        counter += 1
        operation.forward()
        deplacement_left, deplacement_right = calculate_wheel_displacement(motor_left, motor_right)
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        list_pos_x_real.append(pos_x_real* 100)
        list_pos_y_real.append(pos_y_real* 100)
    return pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real

        
def keyBoard_Controller(keyboard, node, robot, operation, motor_left, motor_right, pos_x, pos_y, theta):

    list_pos_x = []
    list_pos_y = []
    list_pos_x_real = []
    list_pos_y_real = []

    deplacement_distance = [0.0, 0.0] # [ left move, Right move ]
    
    while (robot.step(timestep) != -1): #Appel d'une Ã©tape de simulation
        key=keyboard.getKey()
        if (key==ord('Z')) :
            operation.forward()
        elif (key==ord('S')) :
            operation.back()
        elif (key==ord('Q')) :
             operation.rotate_L()
        elif (key==ord('D')) :
             operation.rotate_R()
        elif (key==ord('O')) :
            break
        else:
            operation.stop()
            
        deplacement_left, deplacement_right = calculate_wheel_displacement(motor_left, motor_right)
        
        # position updating
        pos_x, pos_y, theta = update_robot_position(deplacement_left, deplacement_right, pos_x, pos_y, theta)
        pos_x_real, pos_y_real, theta_real = node.getPosition()[2], node.getPosition()[0], node.getOrientation()
        
        list_pos_x.append(pos_x)
        list_pos_y.append(pos_y)
        list_pos_x_real.append(pos_x_real * 100)
        list_pos_y_real.append(pos_y_real * 100)
        
        
        # running distance
        deplacement_distance = (deplacement_distance[0] + deplacement_left, deplacement_distance[1] + deplacement_right)
        
        

        if key==ord('Z') or key==ord('S') or key==ord('Q') or key==ord('D'):
            # print(f'Deplacement for Left Wheel: {deplacement_left}            Deplacement for Right Wheel: {deplacement_right} ')
            print(f'position state : [x={round(pos_x,1)} , y={round(pos_y,1)}]')
            # print(f'Deplacement for left {round(deplacement_distance[0], 1)} cm ; right {round(deplacement_distance[1], 1)} cm')
        update_plot(list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real)
    #difference_plot(list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real)

def map_control(map_node, node, robot, operation, motor_left, motor_right, pos_x, pos_y, theta):
    list_pos_x = []
    list_pos_y = []
    list_pos_x_real = []
    list_pos_y_real = []
    idx = 0

    deplacement_distance = [0.0, 0.0] # [ left move, Right move ]
    
    while (robot.step(timestep) != -1): #Appel d'une Ã©tape de simulation
       
        robot_pos = [pos_x, pos_y]
        robot_theta = theta
       
        target_pos = map_node[idx]
        
        rotation_angle, distance = calculate_movement_parameters(robot_pos, robot_theta, target_pos)
        angle = math.degrees(rotation_angle)
        radio = distance/25
        # print(f'Position(robot): {robot_pos}, Position(target): {target_pos}')
        # print(f'Rotation: {angle}, Advance: {radio}')
        if angle > 0:
            pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real = Robot_L(robot, node, motor_left, motor_right, operation, abs(angle), pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real)
            pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real = Robot_F(robot, node, motor_left, motor_right, operation, radio, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real)
        else:
            pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real = Robot_R(robot, node, motor_left, motor_right, operation, abs(angle), pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real)
            pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real = Robot_F(robot, node, motor_left, motor_right, operation, radio, pos_x, pos_y, theta, list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real)
    

        print(f'position state : [x={round(pos_x,1)} , y={round(pos_y,1)}, theta={round(theta,1)}]')
        idx += 1
        
        if idx % 10 == 0:
            break
         # position updating
    update_plot_map(list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real)
    
    return pos_x, pos_y, theta



def calculate_wheel_displacement(motor_left, motor_right):

    # deplacement = v*dt = w*r*dt
    displacement_left = motor_left.getVelocity() * WHEEL_RADIUS * dt
    displacement_right = motor_right.getVelocity() * WHEEL_RADIUS * dt

    return displacement_left, displacement_right
    


def update_robot_position(displacement_left, deplacement_right, pos_x, pos_y, theta):

    delta_s = (deplacement_right + displacement_left) / 2
    delta_theta = (deplacement_right - displacement_left) / wheel_base

    # Update the robot's orientation
    theta += delta_theta

    # Update the robot's position
    pos_x += delta_s * math.cos(theta + delta_theta / 2)
    pos_y += delta_s * math.sin(theta + delta_theta / 2)
    theta = theta % (2 * math.pi)  # Keep theta within [0, 2Ï€]
    return pos_x, pos_y, theta
    



def update_plot(list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):

    # Interactive mode
    plt.ion()
    plt.plot(list_pos_x, list_pos_y, label='Simulated Robot')
    plt.plot(list_pos_x_real, list_pos_y_real, label='Real Robot')

    plt.plot([0, -50], [0, 0], color='black', linestyle='-', linewidth=5, label='Internal Wall 1')
    plt.plot([0, 0], [50, -50], color='black', linestyle='-', linewidth=5, label='Internal Wall 2')
    plt.plot([0, 50], [50, 50], color='black', linestyle='-', linewidth=5, label='Internal Wall 3')
    
    plt.plot([-25, 75], [75, 75], color='black', linestyle='-', linewidth=2)
    plt.plot([75,75], [75, 25], color='black', linestyle='-', linewidth=2)
    plt.plot([75, 25], [25, 25], color='black', linestyle='-', linewidth=2)
    plt.plot([25, 25], [25, -75], color='black', linestyle='-', linewidth=2)
    plt.plot([25, -25], [-75, -75], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -25], [-75, -25], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -75], [-25, -25], color='black', linestyle='-', linewidth=2)
    plt.plot([-75, -75], [25, -25], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -75], [25, 25], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -25], [25, 75], color='black', linestyle='-', linewidth=2)
    

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.draw()
    plt.pause(0.001)

    plt.clf()
    
def difference_plot(list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):


    # Convert lists to NumPy arrays
    list_pos_x = np.array(list_pos_x)
    list_pos_y = np.array(list_pos_y)
    list_pos_x_real = np.array(list_pos_x_real)
    list_pos_y_real = np.array(list_pos_y_real)

    # Calculate differences
    difference_x = np.abs(list_pos_x - list_pos_x_real)
    difference_y = np.abs(list_pos_y - list_pos_y_real)

    # Set up a figure with two subplots
    fig, axs = plt.subplots(1, 2, figsize=(12, 5))

    # Plot in the first subplot
    axs[0].plot(difference_x, label='Difference X', color='skyblue', linewidth=2)
    axs[0].set_xlabel('Running Step')
    axs[0].set_ylabel('Difference X')
    axs[0].legend()
    axs[0].grid(True)

    # Plot in the second subplot
    axs[1].plot(difference_y, label='Difference Y', color='lightgreen', linewidth=2)
    axs[1].set_xlabel('Running Step')
    axs[1].set_ylabel('Difference Y')
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout(pad=4.0)  # Increase padding for better spacing
    plt.suptitle('Robot Position Differences', fontsize=16)
    plt.show()


def update_plot_map(list_pos_x, list_pos_y, list_pos_x_real, list_pos_y_real):
    # Interactive mode
    # plt.ion()


    plt.plot(list_pos_x, list_pos_y, label='Simulated Robot')
    plt.plot(list_pos_x_real, list_pos_y_real, label='Real Robot')

    plt.plot([0, -50], [0, 0], color='black', linestyle='-', linewidth=5, label='Internal Wall 1')
    plt.plot([0, 0], [50, -50], color='black', linestyle='-', linewidth=5, label='Internal Wall 2')
    plt.plot([0, 50], [50, 50], color='black', linestyle='-', linewidth=5, label='Internal Wall 3')
    
    plt.plot([-25, 75], [75, 75], color='black', linestyle='-', linewidth=2)
    plt.plot([75,75], [75, 25], color='black', linestyle='-', linewidth=2)
    plt.plot([75, 25], [25, 25], color='black', linestyle='-', linewidth=2)
    plt.plot([25, 25], [25, -75], color='black', linestyle='-', linewidth=2)
    plt.plot([25, -25], [-75, -75], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -25], [-75, -25], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -75], [-25, -25], color='black', linestyle='-', linewidth=2)
    plt.plot([-75, -75], [25, -25], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -75], [25, 25], color='black', linestyle='-', linewidth=2)
    plt.plot([-25, -25], [25, 75], color='black', linestyle='-', linewidth=2)
    

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.draw()
    plt.pause(0.001)
    plt.show()

    # plt.clf()



def calculate_movement_parameters(robot_pos, robot_theta, target_pos):
    
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    
    distance = math.sqrt(dx**2 + dy**2)
    angle_to_target = math.atan2(dy, dx)
    
    rotation_angle = angle_to_target - robot_theta
    rotation_angle = (rotation_angle + math.pi) % (2 * math.pi) - math.pi
        
    return rotation_angle, distance


    
         
if __name__=="__main__":
    # size of the square 0.25cm*0.25cm

    robot = Supervisor()
    node = robot.getFromDef("Thymio")

    timestep = int(robot.getBasicTimeStep())
    
    WHEEL_RADIUS = 2.105 # in cm
    TREAD = 10.8 # in cm
    wheel_base = TREAD 
    dt = timestep/1000
    pos_x = node.getPosition()[2]*100  # init x position  
    pos_y = node.getPosition()[0]*100  # init y position  
    theta = 0  # orientation in radians
    
    print(f'Initial Position State : [x={round(pos_x,1)} , y={round(pos_y,1)}, theta={round(theta,1)}]')
    
    # 10 key-nodes of map
    map_node = [
        [-14.2, 12.5],
        [-15.1, 58.4],
        [53.1, 62.2],
        [53.3, 30.5],
        [12.0, 42.8],
        [3.0, -48.9],
        [-20.3, -45.2],
        [-13.5, -9.0],
        [-61.2, -1.0],
        [-56.2, 30.6],
    ]
    

    motor_left = robot.getDevice("motor.left");
    motor_right = robot.getDevice("motor.right");
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    operation = Operation()
    keyboard=Keyboard()
    keyboard.enable(timestep)
    
    # Passive-control by plan
    print("Pre-defined controller activated [####################] 100%")  
    # pos_x, pos_y, theta = map_control(map_node, node, robot, operation, motor_left, motor_right, pos_x, pos_y, theta) 
    print(">> Exit plan control")
    
    # Passive-control by Keyboard
    print("keyboard controller activated [####################] 100%")
    keyBoard_Controller(keyboard, node, robot, operation, motor_left, motor_right, pos_x, pos_y, theta)
    print(">> Exit keyboard control")
    
    
   
    
    
    
    
   
  
        
    

   
    
    


        
        
        
        