#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import picar_4wd as fc
import heapq
import math
import time
import numpy as np
import matplotlib.pyplot as plt
import sign_detector as sd

# Initialize motors with respective GPIO pin numbers
#left_front_motor = Motor(pwm_pin=1, dir_pin=2)
#right_front_motor = Motor(pwm_pin=3, dir_pin=4)
#left_rear_motor = Motor(pwm_pin=5, dir_pin=6)
#right_rear_motor = Motor(pwm_pin=7, dir_pin=8)

# Initialize ultrasonic sensor with trigger and echo pins
#ultrasonic = Ultrasonic(trig=9, echo=10)  # Replace with actual GPIO pin numbers

# Motor speed settings
FORWARD_SPEED = 30
TURN_SPEED = 20
STOP_DISTANCE = 20  # Obstacle distance threshold in cm

# Directions (up, down, left, right, diagonals)
DIRECTIONS = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]

# Define grid world (0 = free, 1 = obstacle)
# 10x10 grid example (can be expanded for real-world use)
#GRID = [
#    [0, 0, 0, 1, 0, 1, 0, 0, 1, 0],
#    [0, 1, 0, 1, 0, 1, 0, 0, 1, 0],
#    [0, 1, 0, 0, 0, 1, 0, 1, 0, 0],
#    [0, 1, 1, 1, 0, 0, 0, 1, 1, 0],
#    [0, 0, 0, 0, 0, 1, 0, 1, 0, 0],
#    [1, 1, 0, 1, 0, 0, 1, 0, 0, 0],
#    [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
#    [0, 1, 0, 0, 0, 1, 1, 1, 1, 0],
#    [0, 1, 1, 1, 0, 0, 0, 1, 0, 0],
#    [0, 0, 0, 1, 0, 1, 0, 1, 0, 0],
#]


def find_Obstacles(angle_step):
    grid=np.zeros((100,100))
    np.int = int
    for i in range(-60,61):
        tmp = int(fc.get_distance_at(i*angle_step))
        x = 49 + np.int(np.sin(np.radians(i*angle_step))*tmp)
        y = np.int(np.cos(np.radians(i*angle_step))*tmp)
        print("X:", x)
        print("Y:", y)
        if abs(x)>99:
            x=99
        if abs(y)>99:
            y=99
        grid[x,y]=1
#        plt.imshow()
    plt.imshow(grid, interpolation="none", origin="lower")
    plt.show()   
    return grid


# A* Pathfinding Algorithm
def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):  # Out of bounds
                continue
            if grid[neighbor[0]][neighbor[1]] == 1:  # Obstacle
                continue

            tentative_g_score = g_score[current] + math.dist(current, neighbor)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Heuristic function (Euclidean distance)
def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

# Reconstruct path from A*
def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]

# Move the PiCar towards the next waypoint in the path
def move_towards_waypoint(current_position, next_position):
    if current_position[0] < next_position[0]:  # Move down
        fc.forward(30)
    elif current_position[0] > next_position[0]:  # Move up
        fc.backward(30)
    elif current_position[1] < next_position[1]:  # Move right
        fc.turn_right(30)
    elif current_position[1] > next_position[1]:  # Move left
        fc.turn_left(30)

# Movement Functions
def move_forward(speed=30):
    left_front_motor.forward(speed)
    right_front_motor.forward(speed)
    left_rear_motor.forward(speed)
    right_rear_motor.forward(speed)

def move_backward(speed=30):
    left_front_motor.backward(speed)
    right_front_motor.backward(speed)
    left_rear_motor.backward(speed)
    right_rear_motor.backward(speed)

def turn_left(speed=20):
    left_front_motor.backward(speed)
    right_front_motor.forward(speed)
    left_rear_motor.backward(speed)
    right_rear_motor.forward(speed)

def turn_right(speed=20):
    left_front_motor.forward(speed)
    right_front_motor.backward(speed)
    left_rear_motor.forward(speed)
    right_rear_motor.backward(speed)

def stop():
    left_front_motor.stop()
    right_front_motor.stop()
    left_rear_motor.stop()
    right_rear_motor.stop()

# Main function for self-driving with path planning
def navigate_with_path(start, goal):

    GRIDS = find_Obstacles(1)
    print("Shape:", GRIDS.shape)
    print("GRID:", GRIDS)
    path = astar(GRIDS, start, goal)
    if path is None:
        print("No path found!")
        return
    
    print(f"Path found: {path}")
    for i in range(1, len(path)):
        current_position = path[i - 1]
        next_position = path[i]
        print(f"Moving from {current_position} to {next_position}")
        move_towards_waypoint(current_position, next_position)
        time.sleep(1)  # Simulate movement delay
        fc.stop()

# Main execution
if __name__ == "__main__":
    try:
        start_pos = (50, 0)  # Start position in the grid
        goal_pos = (85, 85)   # Goal position in the grid

        # Navigate from start to goal using path planning
        navigate_with_path(start_pos, goal_pos)
    except KeyboardInterrupt:
        fc.stop()
