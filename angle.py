import math
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow

def calculate_movement_parameters(robot_pos, robot_theta, target_pos):
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    
    # Calculate the angle towards the target
    angle_to_target = math.atan2(dy, dx)
    
    # Calculate the angle to rotate
    rotation_angle = angle_to_target - robot_theta
    rotation_angle = (rotation_angle + math.pi) % (2 * math.pi) - math.pi
    
    # Calculate the distance to move
    distance = math.sqrt(dx**2 + dy**2)
        
    return rotation_angle, distance

def plot_robot_movement(robot_pos, robot_theta, target_pos):
    rotation_angle, distance = calculate_movement_parameters(robot_pos, robot_theta, target_pos)
    
    # Plotting
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Plot the robot position with label (x, y)
    ax.plot(robot_pos[0], robot_pos[1], 'o', markersize=15, label=f'Robot Position\n({robot_pos[0]}, {robot_pos[1]})', color='limegreen')
    
    # Plot the target position with a marker style 'x' and thicker arrow, with label (x, y)
    ax.scatter(target_pos[0], target_pos[1], marker='x', color='red', s=200, label=f'Target Position\n({target_pos[0]}, {target_pos[1]})')

    # Plot the original orientation angle
    original_angle_arrow = Arrow(robot_pos[0], robot_pos[1], 0.3 * math.cos(robot_theta), 0.3 * math.sin(robot_theta),
                                          color='purple', width=0.12, label='Original Orientation')
    ax.add_patch(original_angle_arrow)
    ax.text(robot_pos[0] + 0.2, robot_pos[1] + 0.1, f'{math.degrees(robot_theta):.2f}°', color='purple', fontsize=12)

    # Plot the rotated orientation angle
    rotated_angle_arrow = Arrow(robot_pos[0], robot_pos[1], 0.3 * math.cos(rotation_angle+robot_theta), 0.3 * math.sin(rotation_angle+robot_theta),
                                         color='orange', width=0.12, label='Rotated Orientation')
    ax.add_patch(rotated_angle_arrow)
    ax.text(robot_pos[0] - 0.1, robot_pos[1] - .3, f'{math.degrees(rotation_angle):.2f}°', color='orange', fontsize=12)

    # Plot the movement vector
    dx = distance * math.cos(rotation_angle+robot_theta)
    dy = distance * math.sin(rotation_angle+robot_theta)
    ax.arrow(robot_pos[0], robot_pos[1], dx, dy, head_width=0.1, head_length=0.15, fc='b', ec='b', label='Movement Vector')

    # Plot a small coordinate axis at the robot position
    ax.quiver(robot_pos[0], robot_pos[1], 0.15 * math.cos(robot_theta), 0.15 * math.sin(robot_theta),
              angles='xy', scale_units='xy', scale=1, color='gray', width=0.05)

    # Set plot limits
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    
    # Add labels and legend
    ax.set_xlabel('X Position', fontsize=14)
    ax.set_ylabel('Y Position', fontsize=14)
    ax.legend(fontsize=12)
    ax.set_title('Robot Movement Visualization', fontsize=16)
    plt.grid()
    plt.show()

if __name__ == "__main__":
    # 示例数据
    robot_pos =  (1, 0.9)
    robot_theta = math.pi/1.2
    target_pos =(1, -1)
    
    # 绘制图表
    plot_robot_movement(robot_pos, robot_theta, target_pos)
