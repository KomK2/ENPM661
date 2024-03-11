import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import cv2
from queue import PriorityQueue
import math

# Constants
y = 500
canvas = np.ones((500, 1200, 3), dtype=np.uint8 )
new_color = (255, 0, 0)
output_file = 'path_video.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
fps = 1000
width, height = 1200,500
video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

def hexagon_vertex() :
    hexagon_vertex = []
    for i in range(6):
        angle_rad = math.radians(30 + 60 * i)  # 60 degrees between each vertex
        x_coordinate = int(650 + 150 * math.cos(angle_rad))
        y_coordinate = int(250 + 150 * math.sin(angle_rad))
        hexagon_vertex.append((x_coordinate, y_coordinate))
    return hexagon_vertex

# Shape definations
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


class Node:
    def __init__(self, x, y,cost = None, parent = None):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
    
    def addCost(self, cost):
        self.cost = cost
    
    def addParent(self, parent):
        self.parent = parent

    def getPoints(self):
        return (self.x,self.y)
    
    def getCost(self):
        return self.cost
    
    def getParent(self):
        return self.parent
    
    def updateCost(self, cost):
        if self.cost is None or cost < self.cost:
            self.cost = cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y 
    
    def __hash__ (self):
        return hash((self.x, self.y))
    
    def __lt__(self, other):
        return self.cost < other.cost

# Function to check if a point is inside a rectangle
def is_in_rectangle(top_left, bottom_right, point):
    if point[0] > top_left[0] and point[0] < bottom_right[0] and point[1] > top_left[1] and point[1] < bottom_right[1]:
        return True
    else:
        return False
    
def point_visualization(point_position):
    cv2.circle(canvas, point_position, 7, (0, 255, 0), -1)

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
    
#Action Space
    
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


def backTracking(node):
    print("Backtracking total cost", node.getCost())
    path = []

    while node is not None:
        # print(node.getPoints())
        path.append(node.getPoints())
        node = node.getParent()

    # print(path)
    path =path[::-1]
    for i in range(1, len(path)):
        # canvas[path[i][1], path[i][0]] = (0, 255, 0)
        cv2.line(canvas, path[i-1], path[i], (0, 255, 0), 2)
        video_writer.write(canvas)



    # video_writer.release()
    plt.imshow(canvas)
    plt.show()
   
    
def dijkstra(start, goal):
    open_list = PriorityQueue()
    my_dict = dict()
    close_list = set()

    # Create the start node
    start_node = Node(start[0], start[1],cost=0)
    open_list.put((0, start_node))
    my_dict[start] = start_node

    while not open_list.empty():
        current_node = open_list.get()[1]


        if current_node.getPoints() == goal:
            print("Goal reached")
            return current_node
        
        temp = my_dict.get(current_node.getPoints())
        if temp is not None:
            if temp.getCost() < current_node.getCost():
                continue
        

        actions = [moveUP, moveDOWN, moveLEFT, moveRIGHT, moveUPLEFT, moveUPRIGHT, moveDOWNLEFT, moveDOWNRIGHT]
    
        for action in actions:
            canMove, newPoints, newCost = action(current_node)
            if canMove:
                possible_next_node = Node(newPoints[0], newPoints[1], cost=newCost+ current_node.getCost() , parent=current_node)
                
                if my_dict.get(newPoints) is None:
                    canvas[newPoints[1], newPoints[0]] = new_color
                    video_writer.write(canvas)
                    my_dict[newPoints] = possible_next_node
                    open_list.put((possible_next_node.getCost(), possible_next_node))

                else:
                    if my_dict[newPoints].getCost() > possible_next_node.getCost():
                        my_dict[newPoints] = possible_next_node
                        open_list.put((possible_next_node.getCost(), possible_next_node))
                        canvas[newPoints[1], newPoints[0]] = new_color
                        video_writer.write(canvas)


if __name__ == "__main__":
    intial, final = start_end_goals()
    goalreached = dijkstra(intial, final)
    backTracking(goalreached)

