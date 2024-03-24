import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import cv2
from queue import PriorityQueue
import math

# Node class
class Node :
    def __init__ (self, points, parent, cost, heuristic):
        self.points = points
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic

    def getPoints(self):
        return self.points
    
    def getParent(self):
        return self.parent
    
    def __eq__(self, other):
        return self.points[0] == other.points[0] and self.points[1] == other.points[1]
    
    def __hash__ (self):
        return hash((self.points[0], self.points[1]))
    
    def __lt__(self, other):
        return self.total_cost < other.total_cost

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

def is_in_rectangle(top_left, bottom_right, point):
    if point[0] > top_left[0] and point[0] < bottom_right[0] and point[1] > top_left[1] and point[1] < bottom_right[1]:
        return True
    else:
        return False
    
def obstacle_space(point):
    # Check if point is inside the hexagon
    hexagon_path = mplPath.Path(hexagon_points)
    if hexagon_path.contains_point(point):
        return True
    # Check if point is inside the rectangle
    if is_in_rectangle((100, y-500), (175, y-100), point):
        return True
    # Check if point is inside the rectangle
    if is_in_rectangle((100+75+100, y-400), (100+75+100+75, y), point):
        return True
    # Check if point is inside the U shape
    u_shape_path = mplPath.Path(u_shape)
    if u_shape_path.contains_point(point):
        return True
    return False

def canMove(point):
    if point[0] < 6 or point[0] > 1195 or point[1] < 6 or point[1] > 495:
        return False 
    point_color = canvas[point[1], point[0]]
    if point_color[0] == 1 and point_color[1] == 1 and point_color[2] == 1:
        return True
    if point_color[0] == 255 and point_color[1] == 0 and point_color[2] == 0:
        return True
    return False


def start_end_goals():
    initial_point = int(input("Enter the x coordinate of the initial point: ")), int(input("Enter the y coordinate of the initial point: "))
    goal_point = int(input("Enter the x coordinate of the goal point: ")), int(input("Enter the y coordinate of the goal point: "))
    if canMove(initial_point) and canMove(goal_point):
        return initial_point, goal_point
    else:
        print("Invalid points. Please enter valid points.")
        return start_end_goals()

def moveUP(node):
    current = node.getPoints()
    newPoints = (current[0],current[1]+1)
    if not canMove(newPoints):
        return False , None ,0 
    return True, newPoints, 1

def moveDOWN(node):
    current = node.getPoints()
    newPoints = (current[0],current[1]-1)
    if not canMove(newPoints):
        return False , None ,0
    return True, newPoints , 1

def moveLEFT(node):
    current = node.getPoints()
    newPoints = (current[0]-1,current[1])
    if not canMove(newPoints):
        return False , None ,0
    return True, newPoints, 1

def moveRIGHT(node):
    current = node.getPoints()
    newPoints = (current[0]+1,current[1])
    if not canMove(newPoints):
        return False , None ,0
    return True, newPoints, 1

def moveUPLEFT(node):
    current = node.getPoints()
    newPoints = (current[0]-1,current[1]+1)
    if not canMove(newPoints):
        return False , None ,0
    return True, newPoints , 1.4

def moveUPRIGHT(node):
    current = node.getPoints()
    newPoints = (current[0]+1,current[1]+1)
    if not canMove(newPoints):
        return False , None,0
    return True, newPoints , 1.4

def moveDOWNLEFT(node):
    current = node.getPoints()
    newPoints = (current[0]-1,current[1]-1)
    if not canMove(newPoints):
        return False , None, 0
    return True, newPoints, 1.4

def moveDOWNRIGHT(node):
    current = node.getPoints()
    newPoints = (current[0]+1,current[1]-1)
    if not canMove(newPoints):
        return False , None ,0
    return True, newPoints, 1.4


def heuristic(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def a_star(initial, final):
    open_list = PriorityQueue()
    closed_list = set()
    visited = dict()

    start_node = Node(initial, None, 0, heuristic(initial, final))

    open_list.put((start_node.total_cost, start_node))
    visited[initial] = start_node
    closed_list.add(initial)
    
    while not open_list.empty():
        current_node = open_list.get()[1]
        current_point = current_node.getPoints()

        if current_point == final:
            path = []
            while current_node is not None:
                path.append(current_node.getPoints())
                canvas[current_node.getPoints()[1], current_node.getPoints()[0]] = (255, 255, 0)
                current_node = current_node.getParent()
            return path[::-1]

        for move in [moveUP, moveDOWN, moveLEFT, moveRIGHT, moveUPLEFT, moveUPRIGHT, moveDOWNLEFT, moveDOWNRIGHT]:
            can_move, new_point, cost = move(current_node)
            if can_move:
                new_node = Node(new_point, current_node, current_node.cost + cost, heuristic(new_point, final))
                if new_point not in closed_list:
                    if new_point not in visited:
                        open_list.put((new_node.total_cost, new_node))
                        visited[new_point] = new_node
                        closed_list.add(new_point)
                        canvas[new_point[1], new_point[0]] = new_color
                    else:
                        if visited[new_point].total_cost > new_node.total_cost:
                            visited[new_point] = new_node
                            open_list.put((new_node.total_cost, new_node))
    




    


if __name__ == "__main__":
    intial, final = start_end_goals()
    a_star(intial, final)
    show_image()
