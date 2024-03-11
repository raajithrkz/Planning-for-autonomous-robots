import numpy as np
import time
import math
import pygame
from heapq import heappush, heappop
import cv2

# Taking inputs from the user
x_start = int(input("Enter the x coordinate for start node (value between 6 and 1194) : "))
y_start = int(input("Enter the y coordinate for start node (value between 6 and 494): "))
x_goal = int(input("Enter the x coordinate for goal node (value between 6 and 1194) : "))
y_goal = int(input("Enter the y coordinate for goal node (value between 6 and 494): "))
#Pack the values in a tuple and initialise clearance
start_point = (y_start, x_start)
goal_point = (y_goal, x_goal)
c = 5
# Boundary coordinates
x_bounds = 1200
y_bounds = 500



def boundary(y_bo, x_bo):
    return (y_bo >= (1 + c) and y_bo <= (y_bounds - c) and x_bo >= (1 + c) and x_bo <= (x_bounds - c))


def all_obstacles(y, x):
    # Verifies if the point lies inside the rectangle shaped obstacle
    # please note that the point coordinates given below are already accounted for the clearance value of 5
    (x1, y1) = (95, 95)
    (x2, y2) = (180, 95)
    (x3, y3) = (180, 495)
    (x4, y4) = (95, 495)
    side1 = y - y1
    side2 = x - x2
    side3 = y - y3
    side4 = x - x4
    rect1 = 1
    if (side1 >= 0 and side2 <= 0 and side3 <= 0 and side4 >= 0):
        rect1 = 0

    # Verifies if the point lies inside the rectangle shaped obstacle
    # please note that the point coordinates given below are already accounted for the clearance value of 5
    (x1, y1) = (270, 5)
    (x2, y2) = (355, 5)
    (x3, y3) = (355, 405)
    (x4, y4) = (270, 405)
    side1 = y - y1
    side2 = x - x2
    side3 = y - y3
    side4 = x - x4
    rect2 = 1
    if (side1 >= 0 and side2 <= 0 and side3 <= 0 and side4 >= 0):
        rect2 = 0
    # Verifies if the point lies inside the hexagon shaped obstacle
    #please note that the point coordinates given below are already accounted for the clearance value of 5
    (x1, y1) = (650,95)
    (x2, y2) = (785, 170)
    (x3, y3) = (785, 330)
    (x4, y4) = (650, 405)
    (x5, y5) = (515, 330)
    (x6, y6) = (515, 170)
    side1 = ((y - y1) * (x2 - x1)) - ((y2 - y1) * (x - x1))
    side2 = ((y - y2) * (x3 - x2)) - ((y3 - y2) * (x - x2))
    side3 = ((y - y3) * (x4 - x3)) - ((y4 - y3) * (x - x3))
    side4 = ((y - y4) * (x5 - x4)) - ((y5 - y4) * (x - x4))
    side5 = ((y - y5) * (x6 - x5)) - ((y6 - y5) * (x - x5))
    side6 = ((y - y6) * (x1 - x6)) - ((y1 - y6) * (x - x6))
    hex = 1
    if (side1 >= 0 and side2 >= 0 and side3 >= 0 and side4 >= 0 and side5 >= 0 and side6 >= 0):
        hex = 0

    # Verifies if the point lies inside the C shaped obstacle
    # please note that the point coordinates given below are already accounted for the clearance value of 5
    (x1, y1) = (855, 45)
    (x2, y2) = (1105, 45)
    (x3, y3) = (1105, 455)
    (x4, y4) = (855, 455)
    (x5, y5) = (855, 370)
    (x6, y6) = (1015, 370)
    (x7, y7) = (1015, 130)
    (x8, y8) = (855, 130)

    # Check if the point is inside the outer C-shape
    inside_c_outer = (x1 <= x <= x2 and y1 <= y <= y4)

    # Check if the point is inside the inner C-shape (the empty space)
    inside_c_inner = (x5 <= x <= x6 and y8 <= y <= y5)

    # The point is inside the C-shape if it's inside the outer shape and not inside the inner shape
    c_shape = inside_c_outer and not inside_c_inner
    rect_u=1
    if c_shape:
        rect_u = 0

    if (rect2 == 0 or hex == 0 or rect1 == 0 or rect_u==0):
        return True
    return False


# Defining the 8 Action Set
def north(y_action, x_action):
    if (boundary(y_action - 1, x_action) and all_obstacles(y_action - 1, x_action) == False):
        return True
    return False

def south(y_action, x_action):
    if (boundary(y_action + 1, x_action) and all_obstacles(y_action + 1, x_action) == False):
        return True
    return False

def east(y_action, x_action):
    if (boundary(y_action, x_action + 1) and all_obstacles(y_action, x_action + 1) == False):
        return True
    return False

def west(y_action, x_action):
    if (boundary(y_action, x_action - 1) and all_obstacles(y_action, x_action - 1) == False):
        return True
    return False

def north_east(y_action, x_action):
    if (boundary(y_action - 1, x_action + 1) and all_obstacles(y_action - 1, x_action + 1) == False):
        return True
    return False

def south_east(y_action, x_action):
    if (boundary(y_action + 1, x_action + 1) and all_obstacles(y_action + 1, x_action + 1) == False):
        return True
    return False

def south_west(y_action, x_action):
    if (boundary(y_action + 1, x_action - 1) and all_obstacles(y_action + 1, x_action - 1) == False):
        return True
    return False

def north_west(y_action, x_action):
    if (boundary(y_action - 1, x_action - 1) and all_obstacles(y_action - 1, x_action - 1) == False):
        return True
    return False


# dijkstra algorithm
def dijkstra():
    # create a hashmap to store distances
    path = {}
    hash_map = {}
    visited_node = {}

    for y in range(1, y_bounds + 1):
        for x in range(1, x_bounds + 1):
            hash_map[(y, x)] = float('inf')
            path[(y, x)] = -1
            visited_node[(y, x)] = False

    # created a queue and marked the initial distance as zero
    explored_node = []
    queue = []
    heappush(queue, (0, start_point))
    hash_map[start_point] = 0

    while (len(queue) > 0):
        _, node = heappop(queue)
        visited_node[node] = True
        explored_node.append(node)

        # if already at the goal node, breaks to quit loop
        if (node[0] == goal_point[0] and node[1] == goal_point[1]):
            break

        # check all possible movements for the current node
        if (west(node[0], node[1]) and visited_node[(node[0], node[1] - 1)] == False and (
                hash_map[(node[0], node[1] - 1)] > hash_map[node] + 1)):
            hash_map[(node[0], node[1] - 1)] = hash_map[node] + 1
            path[(node[0], node[1] - 1)] = node
            heappush(queue, (hash_map[(node[0], node[1] - 1)], (node[0], node[1] - 1)))

        if (east(node[0], node[1]) and visited_node[(node[0], node[1] + 1)] == False and (
                hash_map[(node[0], node[1] + 1)] > hash_map[node] + 1)):
            hash_map[(node[0], node[1] + 1)] = hash_map[node] + 1
            path[(node[0], node[1] + 1)] = node
            heappush(queue, (hash_map[(node[0], node[1] + 1)], (node[0], node[1] + 1)))

        if (north(node[0], node[1]) and visited_node[(node[0] - 1, node[1])] == False and (
                hash_map[(node[0] - 1, node[1])] > hash_map[node] + 1)):
            hash_map[(node[0] - 1, node[1])] = hash_map[node] + 1
            path[(node[0] - 1, node[1])] = node
            heappush(queue, (hash_map[(node[0] - 1, node[1])], (node[0] - 1, node[1])))

        if (south(node[0], node[1]) and visited_node[(node[0] + 1, node[1])] == False and (
                hash_map[(node[0] + 1, node[1])] > hash_map[node] + 1)):
            hash_map[(node[0] + 1, node[1])] = hash_map[node] + 1
            path[(node[0] + 1, node[1])] = node
            heappush(queue, (hash_map[(node[0] + 1, node[1])], (node[0] + 1, node[1])))

        if (south_west(node[0], node[1]) and visited_node[(node[0] + 1, node[1] - 1)] == False and (
                hash_map[(node[0] + 1, node[1] - 1)] > hash_map[node] + 1.4142)):
            hash_map[(node[0] + 1, node[1] - 1)] = hash_map[node] + 1.4142
            path[(node[0] + 1, node[1] - 1)] = node
            heappush(queue, (hash_map[(node[0] + 1, node[1] - 1)], (node[0] + 1, node[1] - 1)))

        if (south_east(node[0], node[1]) and visited_node[(node[0] + 1, node[1] + 1)] == False and (
                hash_map[(node[0] + 1, node[1] + 1)] > hash_map[node] + 1.4142)):
            hash_map[(node[0] + 1, node[1] + 1)] = hash_map[node] + 1.4142
            path[(node[0] + 1, node[1] + 1)] = node
            heappush(queue, (hash_map[(node[0] + 1, node[1] + 1)], (node[0] + 1, node[1] + 1)))

        if (north_east(node[0], node[1]) and visited_node[(node[0] - 1, node[1] + 1)] == False and (
                hash_map[(node[0] - 1, node[1] + 1)] > hash_map[node] + 1.4142)):
            hash_map[(node[0] - 1, node[1] + 1)] = hash_map[node] + 1.4142
            path[(node[0] - 1, node[1] + 1)] = node
            heappush(queue, (hash_map[(node[0] - 1, node[1] + 1)], (node[0] - 1, node[1] + 1)))

        if (north_west(node[0], node[1]) and visited_node[(node[0] - 1, node[1] - 1)] == False and (
                hash_map[(node[0] - 1, node[1] - 1)] > hash_map[node] + 1.4142)):
            hash_map[(node[0] - 1, node[1] - 1)] = hash_map[node] + 1.4142
            path[(node[0] - 1, node[1] - 1)] = node
            heappush(queue, (hash_map[(node[0] - 1, node[1] - 1)], (node[0] - 1, node[1] - 1)))

    # return if no optimal path was found
    if (hash_map[goal_point] == float('inf')):
        return (explored_node, [], hash_map[goal_point])

    # backtrack path
    back_track = []
    node = goal_point
    while (path[node] != -1):
        back_track.append(node)
        node = path[node]
    back_track.append(start_point)
    back_track = list(reversed(back_track))
    return (explored_node, back_track, hash_map[goal_point])


# animate node exploration and backtracking
def animater(explored_node, back_track, path):
    f = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(str(path), f, 20.0, (x_bounds, y_bounds))
    field = np.zeros((y_bounds, x_bounds, 3), dtype=np.uint8)
    count = 0
    for state in explored_node:
        field[int(y_bounds - state[0]), int(state[1] - 1)] = (200, 255, 200) #highlights explored area
        if (count % 100 == 0):
            out.write(field)
        count = count + 1

    count = 0
    for y in range(1, y_bounds + 1):
        for x in range(1, x_bounds + 1):
            if (field[int(y_bounds - y), int(x - 1), 0] == 0 and field[int(y_bounds - y), int(x - 1), 1] == 0 and field[
                int(y_bounds - y), int(x - 1), 2] == 0):
                if (boundary(y, x) and all_obstacles(y, x) == False):
                    field[int(y_bounds - y), int(x - 1)] = (0, 0, 255) #highlights unexplored area
                    if (count % 100 == 0):
                        out.write(field)
                    count = count + 1

    if (len(back_track) > 0):
        for state in back_track:
            field[int(y_bounds - state[0]), int(state[1] - 1)] = (255, 0, 0) #highlights backtracking line
            out.write(field)
            cv2.imshow('result', field)
            cv2.waitKey(5)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
    out.release()


# main code
if (boundary(start_point[0], start_point[1])):
    if (boundary(goal_point[0], goal_point[1])):
        if (all_obstacles(start_point[0], start_point[1]) == False):
            if (all_obstacles(goal_point[0], goal_point[1]) == False):
                (explored_node, back_track, optimal) = dijkstra()
                animater(explored_node, back_track, "./dijkstra.avi")
                print("\nThe Optimal distance is: " + str(optimal))
            else:
                print("The goal coordinates you inputted lie inside an obstacle")
        else:
            print("The initial coordinates you inputted lie inside an obstacle")
    else:
        print("The goal coordinates you inputted are outside the map")
else:
    print("The initial coordinates you inputted are outside the map")