# ENPM 661 - Planning for Autonomous Robots
# Project 3-2 : Implementing A* Algorithm 
# Author : Kiran Kommaraju, Abhinav Bhamidipati
# UID : komkiran, abhinav7

# Import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as mplPath
import cv2
from queue import PriorityQueue
import math

# Node class
class Node :
    def __init__ (self, points, orientation,  parent, cost, heuristic):
        self.points = points
        self.orientation = orientation
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.total_cost = cost + heuristic

    # Points of the Node
    def getPoints(self):
        return self.points
    
    # Orientation of the Node
    def getOrientation(self):
        return self.orientation
    
    # Parent Node
    def getParent(self):
        return self.parent
    
    # Check if two nodes are equal
    def __eq__(self, other):
        return self.points[0] == other.points[0] and self.points[1] == other.points[1]
    
    def __hash__ (self):
        return hash((self.points[0], self.points[1]))
    
    def __lt__(self, other):
        return self.total_cost < other.total_cost

# Constants
y = 2000
goal = None
start_point = None
canvas = np.ones((2000, 6000, 3), dtype=np.uint8 )

# color for exploration of nodes
new_color = (255, 0, 0)


left_rpm =0
right_rpm = 0

# robot_radius = 38  
robot_wheel_radius = 30
wheel_distance = 354  
dt = 0.3
t = 0


# Video writer to save the output
# output_file = 'path_video.avi'
# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# fps = 1000
# width, height = 1200,500
# video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

# Initializing Variables
def initalize_varaibles():
    global right_rpm
    global left_rpm


    right_rpm = int (input("enter right wheel rpm : "))
    left_rpm = int(input("enter left wheel rpm : "))


# Rectangle 1
# cv2.rectangle(canvas, (1500, y-2000), (1750, y-1000), (255, 0, 255), 175)
cv2.rectangle(canvas, (1500, y-2000), (1750, y-1000), (0, 0, 255), -1)

# Rectangle 2
# cv2.rectangle(canvas, (2500, y-1000), (2750, y-0), (255, 0, 255), 175)
cv2.rectangle(canvas, (2500, y-1000), (2750, y-0), (0, 0, 255), -1)

# Circle shape
# cv2.circle(canvas, (4200, y-1200), 600, (255, 0, 255), 175)
cv2.circle(canvas, (4200, y-1200), 600, (0, 0, 255), -1)


# display the canvas
def show_image():
    plt.imshow( canvas)
    plt.show()

# check if point is inside the rectangle
def is_in_rectangle(top_left, bottom_right, point):
    if point[0] > top_left[0] and point[0] < bottom_right[0] and point[1] > top_left[1] and point[1] < bottom_right[1]:
        return True
    else:
        return False
    
def is_in_circle(center, radius, point):
    if (point[0] - center[0])**2 + (point[1] - center[1])**2 < radius**2:
        return True
    else:
        return False

# Check if point is inside the obstacle space    
def obstacle_space(point):
    # Check if point is inside the rectangle
    if is_in_rectangle((1500-150, y-2000), (1750+150, y-(1000 +150)), point):
        return True
    # Check if point is inside the rectangle
    if is_in_rectangle((2500-150, y-1000), (2750, y-150), point):
        return True
    # Check if point is inside the Circle
    if is_in_circle((4200, y-1200), 600+150, point):
        return True
    return False

# Checking if the point can move
def canMove(point):
    if point[0] < 6 or point[0] > 5995 or point[1] < 6 or point[1] > 1995:
        return False 
    if ((point[0] > (1500-100)) and (point[0] <(1750+100) ) and ((point[1] > 0) and (point[1] <(1000+100)) )):
        return False
    if ((point[0] > (2500-100)) and (point[0] <(2750+100) ) and ((point[1] < 2000) and (point[1] >(1000-100)))):
        return False
    if is_in_circle((4200, y-1200), 600+125, point):
        return False
    point_color = canvas[point[1], point[0]]
    if point_color[0] == 1 and point_color[1] == 1 and point_color[2] == 1:
        return True
    if point_color[0] == 255 and point_color[1] == 0 and point_color[2] == 0:
        return True
    return False

# Start and End points from the user
#<!--Note the star and end goals should be in radians per second>
def start_end_goals():
    # initial_point = int(input("Enter the x coordinate of the initial point: ")), int(input("Enter the y coordinate of the initial point: "))
    # inital_orentation = int(input("Enter the orientation of robot at initial point ( multiple of 30 ): "))

    initial_point = (500,1000)
    inital_orentation = math.radians(60)

    goal_point = (5500, 1000)
    goal_orentation = math.radians(0)

    # goal_point = int(input("Enter the x coordinate of the goal point: ")), int(input("Enter the y coordinate of the goal point: "))
    # goal_orentation = int(input("Enter the orientation of robot at goal point ( multiple of 30 ): "))

    if canMove(initial_point) and canMove(goal_point):
        return initial_point, goal_point , inital_orentation, goal_orentation
    else:
        print("Invalid points. Please enter valid points.")
        return start_end_goals()
    
def possible_increment(left_wheel_rpm, right_wheel_rpm,theta):
    left_wheel_rpm = ((2*np.pi)*left_wheel_rpm)/60
    right_wheel_rpm = ((2*np.pi)*right_wheel_rpm)/60

    dt = 0.5
    dx = 0.5*robot_wheel_radius*(left_wheel_rpm+right_wheel_rpm)*(math.cos(theta))*dt
    dy = 0.5*robot_wheel_radius*(left_wheel_rpm+right_wheel_rpm)*(math.sin(theta))*dt
    dtheta = (robot_wheel_radius/wheel_distance)*(right_wheel_rpm - left_wheel_rpm)*dt
    return dx , dy , dtheta


def move(left, right,node2, end_point):
    current_node = node2
    node_list = []

    for i in range(8):
        current_points = current_node.getPoints()
        current_orentation = current_node.getOrientation()

        dx , dy , dtheta = possible_increment(left,right, current_orentation)

        new_x = int(round(current_points[0] + dx))
        new_y = int(round(current_points[1] + dy))
        new_theta = current_orentation + dtheta

        move_cost = math.sqrt((dx)**2 + (dy)**2)
        new_points = (new_x,new_y)

        if not canMove(new_points):
            return 
        new_node = Node(points=new_points,orientation=new_theta, parent= current_node , cost= current_node.cost +move_cost , heuristic= heuristic((new_x, new_y),end_point))
        node_list.append(new_node)
        current_node = new_node

    for sub_node in node_list:
        cv2.line(canvas, sub_node.getPoints(), sub_node.getParent().getPoints(), new_color, 1)

    return node_list

def action1(node1):
    return move( left=0, right= left_rpm, node2= node1, end_point=final)

def action2(node1):
    return move( left=left_rpm, right= 0, node2= node1, end_point=final)

def action3(node1):
    return move( left=left_rpm, right= left_rpm, node2= node1, end_point=final)

def action4(node1):
    return move( left=0, right= right_rpm, node2= node1, end_point=final)

def action5(node1):
    return move( left=right_rpm, right= 0, node2= node1, end_point=final)

def action6(node1):
    return move( left=right_rpm, right= right_rpm, node2= node1, end_point=final)

def action7(node1):
    return move( left=left_rpm, right= right_rpm, node2= node1, end_point=final)

def action8(node1):
    return move( left=right_rpm, right= left_rpm, node2= node1, end_point=final)


# Check if point is in the goal threshold
def in_goal_thershold(point,goal, thershold):
    x , y = point[0] ,point[1]
    circle_center_x, circle_center_y, = goal[0], goal[1]
    radius = thershold
    distance = math.sqrt((x - circle_center_x) ** 2 + (y - circle_center_y) ** 2)
    return distance <= radius

# Calculate the heuristic value = Eulicdean distance
def heuristic(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# A* Algorithm 
def a_star(initial, final, inital_orentation ):
    # Initialize lists
    open_list = PriorityQueue()
    closed_list = set()
    visited = dict()

    start_node = Node(initial, inital_orentation ,None, 0, heuristic(initial, final))

    open_list.put((start_node.total_cost, start_node))
    visited[initial] = start_node
    closed_list.add(initial)
    
    count =0
    while not open_list.empty():
        count = count +1

        current_node = open_list.get()[1]
        current_point = current_node.getPoints()

        # Check if current point is within the goal threshold
        if in_goal_thershold(current_point,final, 100):
            print("Goal reached")
            path = []
            while current_node is not None:
                path.append(current_node.getPoints())
                canvas[current_node.getPoints()[1], current_node.getPoints()[0]] = (6, 142, 175)
                current_node = current_node.getParent()
                
            return path[::-1]

        # Move in all directions
        for move in [action1, action2, action3, action4, action5, action6, action7, action8]:
            one_move_sub_nodes = move(current_node)
            if one_move_sub_nodes is not None:
                new_node = one_move_sub_nodes[len(one_move_sub_nodes)-1]
                new_point = new_node.getPoints()
                if new_point not in closed_list:
                    if new_point not in visited:
                        open_list.put((new_node.total_cost, new_node))
                        visited[new_point] = new_node
                        closed_list.add(new_point)
                        
                        if count %1000 == 0:
                            resized_canvas = cv2.resize(canvas, (1200, 400))
                            cv2.imshow("abhinav1" ,resized_canvas)
                            cv2.waitKey(1)
                        # video_writer.write(canvas)
                    else:
                        if visited[new_point].total_cost > new_node.total_cost:
                            visited[new_point] = new_node
                            open_list.put((new_node.total_cost, new_node))

# Draw the path as arrows 
def draw_arrow(path):
                    
    for i in range(len(path) - 1):
        start_node = path[i]
        end_node = path[i + 1]
        cv2.arrowedLine(canvas, start_node, end_node, (0, 255, 0), 5 , tipLength= 0.5)
        # video_writer.write(canvas)
        

# Main function
if __name__ == "__main__":
    intial, final, inital_orentation, goal_orentation = start_end_goals()
    initalize_varaibles()
    print("Exploring the world. Please wait !!")
    shortest_path = a_star(intial, final,inital_orentation)
    
    # # Visualize start and goal nodes
    cv2.circle(canvas, (intial[0], intial[1]), 5, (0, 255, 0), -1)  # Start node in white
    cv2.circle(canvas, (final[0], final[1]), 5, (0, 255, 0), -1)  # Goal node in green

    draw_arrow(shortest_path)
    # video_writer.release()
    # print(f"can move : {canMove((4167,736))}")
    show_image()
