import picar_4wd as fc
import sys
import signal
import numpy as np
import time
import threading
from car_mapping import Car
from astar_algo import Node
from astar_algo import backtrace, manhattan_heuristic, compute_neighbors, astar_search

MAX_PATH_LEN = 6
RIGHT_DIR = 0
UP_DIR = 90
LEFT_DIR = 180
DOWN_DIR = 270
POWER_VAL = 3
STEP = 1

RIGHT_BOUND = 99
LEFT_BOUND = -100
UP_BOUND = 99
DOWN_BOUND = 0
X_EDGE_COORD = 100
Y_EDGE_COORD = 99

thread1 = None
thread2 = None

car_stop_status = False

# function which allows you to stop the car given a certain signal
def car_stop(signal, frame):
    global car_stop_status
    car_stop_status = True

# function which allows you to start the car given a certain signal
def car_start(signal, frame):
    global car_stop_status
    car_stop_status = False

def turn_car(car, prev_node, curr_node):
    horiz_dist = curr_node[0] - prev_node[0]
    vert_dist = curr_node[1] - prev_node[1]

    if horiz_dist == -1:
        car.map_orientation(LEFT_DIR, POWER_VAL)
    elif horiz_dist == 1:
        car.map_orientation(RIGHT_DIR, POWER_VAL)
    elif vert_dist == -1:
        car.map_orientation(UP_DIR, POWER_VAL)
    elif vert_dist == 1:
        car.map_orientation(DOWN_DIR, POWER_VAL)

    car.move_distance(STEP, POWER_VAL)

def navigatePath(path, target_dest, mutex, car):
    # we can return if there is no path..
    if path is None:
        return None

    moves_made = (0, 0)
    prev_node = None

    for path_idx, curr_node in enumerate(path):
        if prev_node == target_dest:
            break
        elif path_idx == MAX_PATH_LEN:
            break
        mutex.acquire(blocking=True)

        # case where we have only the first node processed
        if prev_node is None:
            prev_node = curr_node
            mutex.release()
            continue

        # make a move based on current and previous nodes
        moves_made = (moves_made[0] + curr_node[0] - prev_node[0], moves_made[1] + curr_node[1] - prev_node[1])

        # based on coordinates, we need to turn and move in the appropriate direction
        turn_car(car, prev_node, curr_node)

        # at the end of this current move, we need to update the previous node value for the next iteration
        prev_node = curr_node
        mutex.release()

    return moves_made

def navigateToPoint(x_coord, y_coord, car, mutex):
    while x_coord != 0 or y_coord != 0:
        if x_coord > RIGHT_BOUND:
            remaining_x_coord = x_coord - RIGHT_BOUND
            x_coord = RIGHT_BOUND
        elif x_coord < LEFT_BOUND:
            remaining_x_coord = x_coord + LEFT_BOUND
            x_coord = LEFT_BOUND

        x_coord = X_EDGE_COORD + x_coord

        if y_coord > UP_BOUND:
            remaining_y_coord = y_coord - UP_BOUND
            y_coord = UP_BOUND
        elif y_coord < DOWN_BOUND:
            y_coord = DOWN_BOUND + y_coord

        y_coord = Y_EDGE_COORD + y_coord

        curr_move_coord = (int(car.get_x()), int(car.get_y()))
        moves_toward_point = (0, 0)

        while (x_coord, y_coord) != curr_move_coord:
            car.mapping()
            maze = car.get_env_grid()
            route = astar_search(maze, curr_move_coord, (x_coord, y_coord))
            moves_toward_point = navigatePath(route, (x_coord, y_coord), car, mutex)

            curr_move_coord = (curr_move_coord[0] + moves_toward_point[0], curr_move_coord[1] + moves_toward_point[1])

        x_coord = remaining_x_coord
        y_coord = remaining_y_coord

def main():
    signal.signal(signal.SIGUSR1, stop_car)
    signal.signal(signal.SIGUSR2, stop_car)
    car = Car()
    mutex = threading.Lock()

    # We want to introduce threading to speed up the utilization of resources
    # Which will in turn help the car move faster
    thread1 = threading.Thread(target=navigateToPoint, args=(0, 10, car, mutex))
    thread2 = threading.Thread(target=run_object_detection, args=(mutex,))

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

if __name__ == "__main__":
    main()