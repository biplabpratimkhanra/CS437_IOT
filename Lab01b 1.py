import picar_4wd as fc
import math
import time
import matplotlib.pyplot as plt
import numpy as np
import heapq

########################################################
# Ultrasonic
ANGLE_RANGE = 30
STEP = 10
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []

errors = []

def scan_step(ref):
    global scan_list, current_angle, us_step
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP
    status = fc.get_status_at(current_angle, ref1=ref)#ref1

    scan_list.append(status)
    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            # print("reverse")
            scan_list.reverse()
        # print(scan_list)
        tmp = scan_list.copy()
        scan_list = []
        return tmp
    else:
        return False


def find_Obstacles(angle_step):
    grid=np.zeros((100,100))
    np.int = int
    for i in range(-90,90):
        tmp = int(fc.get_distance_at(i*angle_step))
        if tmp < 0:
            tmp = 0
        print("Angle:/ Distance", i, tmp)
        x = 49 + np.int(np.sin(np.radians(i*angle_step))*tmp)
        y = np.int(np.cos(np.radians(i*angle_step))*tmp)
        print("X:", x)
        print("Y:", y)
        if abs(x)>99:
            x=99
        if abs(y)>99:
              y=99
        grid[x,y]=1

    plt.imshow(grid, interpolation="none", origin="lower")
    plt.show()   
    return grid



# Define the heuristic function (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* algorithm to find the shortest path
def astar(maze, start, goal):
    # Priority queue for the open set
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        neighbors = get_neighbors(maze, current)
        for neighbor in neighbors:
            tentative_g_score = g_score[current] + 1  # Distance between nodes is 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Get the neighbors of a given position
def get_neighbors(maze, pos):
    neighbors = []
    rows, cols = maze.shape
    row, col = pos

    # Check the four possible directions (up, down, left, right)
    if row > 0 and maze[row - 1][col] == 0:  # Up
        neighbors.append((row - 1, col))
    if row < rows - 1 and maze[row + 1][col] == 0:  # Down
        neighbors.append((row + 1, col))
    if col > 0 and maze[row][col - 1] == 0:  # Left
        neighbors.append((row, col - 1))
    if col < cols - 1 and maze[row][col + 1] == 0:  # Right
        neighbors.append((row, col + 1))

    return neighbors

# Convert path to movements
def get_moves(path):
    moves = []
    # Define orientation: 0 -> facing right, 1 -> facing down, 2 -> facing left, 3 -> facing up
    orientation = 0  # Start facing right
    directions = {
        (0, 1): 'right',
        (0, -1): 'left',
        (1, 0): 'down',
        (-1, 0): 'up'
    }
    
    for i in range(len(path) - 1):
        # Calculate movement direction
        current = path[i]
        next_step = path[i + 1]
        move = (next_step[0] - current[0], next_step[1] - current[1])
        desired_direction = directions[move]
        
        if desired_direction == 'right':
            if orientation == 1:  # Facing down
                moves.append('left')
            elif orientation == 2:  # Facing left
                moves.append('turn around')
            elif orientation == 3:  # Facing up
                moves.append('right')
            orientation = 0  # Now facing right
        elif desired_direction == 'down':
            if orientation == 0:  # Facing right
                moves.append('right')
            elif orientation == 2:  # Facing left
                moves.append('left')
            elif orientation == 3:  # Facing up
                moves.append('turn around')
            orientation = 1  # Now facing down
        elif desired_direction == 'left':
            if orientation == 0:  # Facing right
                moves.append('turn around')
            elif orientation == 1:  # Facing down
                moves.append('right')
            elif orientation == 3:  # Facing up
                moves.append('left')
            orientation = 2  # Now facing left
        elif desired_direction == 'up':
            if orientation == 0:  # Facing right
                moves.append('left')
            elif orientation == 1:  # Facing down
                moves.append('turn around')
            elif orientation == 2:  # Facing left
                moves.append('right')
            orientation = 3  # Now facing up
        
        moves.append('up')
    
    return moves


# PiCar-4WD Control Functions
def move_picar(direction):
    print("Directions :", direction)
    if direction == "up":
        print("Moving forward")
        fc.forward(20)
        time.sleep(.1)
        # Add PiCar-4WD API call to move forward
    elif direction == "down":
        print("Moving backward")
        # Add PiCar-4WD API call to move backward
        fc.backward(20) 
        time.sleep(.1)
    elif direction == "left":
        print("Turning left")
        # Add PiCar-4WD API call to turn left
        fc.turn_left(20)
        time.sleep(1.5)
    elif direction == "right":
        print("Turning righ")
        # Add PiCar-4WD API call to turn right
        fc.turn_right(20)
        time.sleep(1.5)
    elif direction == "turn around":
        print("Turning around (180 degrees)")
        fc.turn_right(20)
        time.sleep(1.5)  # Adjust timing for 180-degree turn
        fc.turn_right(20)
        time.sleep(1.5)
    fc.stop()
# Simulate maze and run A* with PiCar-4WD

def move(start,goal):
    move_ct=0
    maze = find_Obstacles(1)
    
    # Get the path using A*
    path = astar(maze, start, goal)
    if path:
        print("Path found:", path)
        # Convert path to moves
        moves = get_moves(path)
        print("Moves:", moves)
        # Control PiCar-4WD based on the moves
        for move in moves:
            #if ():# Add the Image recognition element here 
             #   fc.stop(5)
                
            scan_list = scan_step(10)
            
            if not scan_list:
                continue
            
            print('Scan list is ',scan_list[1])
            
            if scan_list[1]!=2:#Obstacle Detected
                print("New Obstacle Detected... Printing the path sofar and Replanning")
                new_start=(int(x) for x in np.asarray(path[move_ct]))
                #move(new_start,goal)
            else: 
                move_picar(move)
                print('The position is ',path[move_ct+1])
                move_ct=move_ct+1            
            
            
        plt.imshow(maze, interpolation="none", origin="lower")
        for step in path:
            plt.plot(step[1], step[0], 'ro')  # Plot the path as red dots
        plt.show()
        fc.stop()
    else:
        print("No path found!")




def main():

    start = (50, 00)
    goal = (20, 60)
    move(start,goal)
    
if __name__ == "__main__":
    main()
    
    
    
