import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import cv2
from queue import PriorityQueue
import math
from node import Node



# Constants
y = 500
goal = None
start_point = None
canvas = np.ones((500, 1200, 3), dtype=np.uint8 )
new_color = (255, 0, 0)

def hexagon_vertex() :
    hexagon_vertex = []
    for i in range(6):
        angle_rad = math.radians(30 + 60 * i)  # 60 degrees between each vertex
        x_coordinate = int(650 + 150 * math.cos(angle_rad))
        y_coordinate = int(250 + 150 * math.sin(angle_rad))
        hexagon_vertex.append((x_coordinate, y_coordinate))
    return hexagon_vertex

# Shape definitions
hexagon_points = np.array(hexagon_vertex())
u_shape = np.array([[1200-100-200,y-(450)],[1200-100,y-450],[1200-100,y-50],[1200-100-200,y-(50)],[1200-100-200,y-75-50],[1200-100-200+120,y-75-50],[1200-100-200+120,y-(375)],[1200-100-200,y-375] ])

cv2.polylines(canvas, [hexagon_points], True, (255, 0, 255), 5)
cv2.fillPoly(canvas, [hexagon_points], (0, 0, 255))

cv2.rectangle(canvas, (100, y-500), (175, y-100), (255, 0, 255), 5 )
cv2.rectangle(canvas, (100, y-500), (175, y-100), (0, 0, 255), -1)

cv2.rectangle(canvas, (100+75+100, y-400), (100+75+100+75, y), (255, 0, 255), 5)
cv2.rectangle(canvas, (100+75+100, y-400), (100+75+100+75, y), (0, 0, 255), -1)

cv2.polylines(canvas, [u_shape], True, (255, 0, 255), 5)
cv2.fillPoly(canvas, [u_shape],(0, 0, 255))

def show_image():
    plt.imshow( canvas)
    plt.show()

def heuristic(current):
    distance_sq = ((goal[0]-current[0])**2) + ((goal[1]-current[1])**2)
    return math.sqrt(distance_sq) 

def can_move(point):
    if point[0] < 6 or point[0] > 1195 or point[1] < 6 or point[1] > 495:
        return False 
    point_color = canvas[point[1], point[0]]
    if point_color[0] == 1 and point_color[1] == 1 and point_color[2] == 1:
        return True
    if point_color[0] == 255 and point_color[1] == 0 and point_color[2] == 0:
        return True
    return False


def move_30_left(node, stepsize = 1):
    current_point = node.getPoints()
    new_points = (round(current_point[0]+ (np.cos(-np.pi/6) *stepsize)),round( current_point[1]+ (np.sin( -np.pi/6) *stepsize)))
    if can_move(new_points):
        return True, new_points, node.orientation -30, heuristic(new_points)
    return False , current_point, node.orientation, node.heuristic
    

def move_60_left(node, stepsize = 1):
    current_point = node.getPoints()
    new_points = (round(current_point[0]+ (np.cos(-np.pi/3) *stepsize)) , round(current_point[1]+ (np.sin(-np.pi/3) *stepsize)))
    if can_move(new_points):
        return True, new_points,node.orientation -60, heuristic(new_points)
    return False , current_point, node.orientation, node.heuristic

def move_straight(node, stepsize = 1):
    current_point = node.getPoints()
    new_points = (round(current_point[0]+ (np.cos(0) *stepsize)) , round(current_point[1]+ (np.sin(0) *stepsize)))
    if can_move(new_points):
        return True, new_points,node.orientation , heuristic(new_points)
    return False , current_point, node.orientation, node.heuristic

def move_30_right(node, stepsize = 1):
    current_point = node.getPoints()
    new_points = (round(current_point[0]+ (np.cos(np.pi/6) *stepsize) ), round(current_point[1]+ (np.sin(np.pi/6) *stepsize)))
    if can_move(new_points):
        return True, new_points,node.orientation +30 , heuristic(new_points)
    return False , current_point, node.orientation, node.heuristic

def move_60_right(node, stepsize = 1):
    current_point = node.getPoints()
    new_points = (round(current_point[0]+ (np.cos(np.pi/3) *stepsize) ), round(current_point[1]+ (np.sin(np.pi/3) *stepsize)))
    if can_move(new_points):
        return True, new_points,node.orientation +60, heuristic(new_points)
    return False , current_point, node.orientation, node.heuristic

def in_goal_thershold(point, thershold):
    x , y = point[0] ,point[1]
    circle_center_x, circle_center_y, = goal[0], goal[1]
    radius = thershold
    distance = math.sqrt((x - circle_center_x) ** 2 + (y - circle_center_y) ** 2)
    return distance <= radius



def a_star_implementation():
    open_list = PriorityQueue()
    visted_list = dict()

    first_node = Node(start_point[0],start_point[1],start_point[2],cost=0, heuristic =  heuristic(start_point), parent=None)
    
    open_list.put((first_node.total_cost ,first_node))
    visted_list[start_point] = first_node

    while True :
        current_node =  open_list.get()[1]
        current_node_location = (current_node.x,current_node.y)
        print(f"points are {current_node_location} and orientation : {current_node.orientation}",)
        if in_goal_thershold(current_node_location, 1):
            print("goal reached")
            plt.imshow(canvas)
            plt.show()
            return 
        
        
        actions = [move_30_left, move_60_left, move_straight, move_30_right, move_60_right]
        for action in actions:
            canMove, newPoints,newOrientation, newHeuristic = action(current_node, stepsize= 2)

            if canMove:
                possibile_next_node = Node(newPoints[0],newPoints[1],newOrientation ,1 ,newHeuristic, parent= current_node)

                if visted_list.get(newPoints) :
                    if newHeuristic < visted_list[newPoints].heuristic :
                        visted_list[newPoints] = possibile_next_node
                        canvas[newPoints[1], newPoints[0]] = new_color
                else:
                    visted_list[newPoints] = possibile_next_node
                    open_list.put((newHeuristic,possibile_next_node))
                    canvas[newPoints[1], newPoints[0]] = new_color


if __name__ == "__main__":
    x_start, y_start, start_orientation = map(int, input("Enter start x, start y, and start orientation separated by spaces: ").split())
    start_point = (x_start, y_start, start_orientation)

    x_goal, y_goal, goal_orientation = map(int, input("Enter goal x, goal y, and goal orientation separated by spaces: ").split())
    goal = (x_goal, y_goal, goal_orientation)
    a_star_implementation()


    # show_image()