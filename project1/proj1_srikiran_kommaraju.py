import numpy as np
from collections import deque

config = {}
"""All configurations are stored in a dictionary to avoid visiting the same node again."""

class Node:
    """
    Represents a node in the puzzle.

    Attributes:
    - parent: The parent node.
    - map: A dictionary representing the current state of the puzzle.
    - index: The index of the node.
    """

    def __init__(self, parent=None):
        """
        Initializes a new Node object.

        Parameters:
        - parent: The parent node (default is None).
        """
        self.parent = parent
        self.map = {}

    def map_indexes(self, state):
        """
        Maps the indexes of the puzzle state.

        Parameters:
        - state: The puzzle state.
        """
        for i in range(len(state)):
            for j in range(len(state[i])):
                value = state[i][j]
                self.map[value] = (i, j)
    
    def getKey(self, x, y):
        """
        Gets the key from the map based on the given coordinates.

        Parameters:
        - x: The x-coordinate.
        - y: The y-coordinate.

        Returns:
        - The key corresponding to the given coordinates.
        """
        for key, value in self.map.items():
            if value == (x, y):
                return key
    
    def addMap(self, map):
        """
        Adds a map to the node.

        Parameters:
        - map: The map to be added.
        """
        self.map = map

    def __eq__(self, other):
        """
        Checks if two nodes are equal.

        Parameters:
        - other: The other node to compare.

        Returns:
        - True if the nodes are equal, False otherwise.
        """
        if not isinstance(other, Node):
            return NotImplemented

        return (self.map == other.map)
    
    def __hash__(self):
        """
        Computes the hash value of the node.

        Returns:
        - The hash value of the node.
        """
        items_tuple = tuple(sorted(self.map.items()))
        return hash(items_tuple)
    
    def addIndex(self, index):
        """
        Adds an index to the node.

        Parameters:
        - index: The index to be added.
        """
        self.index = index
    
    def getIndex(self):
        """
        Gets the index of the node.

        Returns:
        - The index of the node.
        """
        return self.index
        
    
def ActionMoveDown(node):
    """
    Moves the empty space down in the puzzle.

    Parameters:
    - node: The current node.

    Returns:
    - The new node after moving down.
    - True if it is possible to move down, False otherwise.
    """
    if node.map[0][0] == 2:
        return node, False
    else:
        new_map = node.map.copy()
        current_index_of_zero = (node.map[0][0], node.map[0][1])
        
        swapping_key = node.getKey(node.map[0][0] + 1, node.map[0][1])
        new_map[0] = (node.map[0][0] + 1, node.map[0][1])
        new_map[swapping_key] = current_index_of_zero
        immutable_dict_key = tuple(sorted(new_map.items()))

        if config.get(immutable_dict_key) is None:
            config[immutable_dict_key] = 1
            newNode = Node(node)
            newNode.addMap(new_map)
            newNode.addIndex(node.getIndex() + 1)
            return newNode, True
        else:
            return node, True
    
def ActionMoveUp(node):
    """
    Moves the empty space up in the puzzle.

    Parameters:
    - node: The current node.

    Returns:
    - The new node after moving up.
    - True if it is possible to move up, False otherwise.
    """
    if node.map[0][0] == 0:
        return node, False
    else:
        new_map = node.map.copy()
        current_index_of_zero = (node.map[0][0], node.map[0][1])
        
        swapping_key = node.getKey(node.map[0][0] - 1, node.map[0][1])
        new_map[0] = (node.map[0][0] - 1, node.map[0][1])
        new_map[swapping_key] = current_index_of_zero

        immutable_dict_key = tuple(sorted(new_map.items()))
        if config.get(immutable_dict_key) is None:
            config[immutable_dict_key] = 1
            newNode = Node(node)
            newNode.addMap(new_map)
            newNode.addIndex(node.getIndex() + 1)
            return newNode, True
        else:
            return node, False
        
def ActionMoveRight(node):
    """
    Moves the empty space right in the puzzle.

    Parameters:
    - node: The current node.

    Returns:
    - The new node after moving right.
    - True if it is possible to move right, False otherwise.
    """
    if node.map[0][1] == 2:
        return node, False
    else:
        new_map = node.map.copy()
        current_index_of_zero = (node.map[0][0], node.map[0][1])
        
        swapping_key = node.getKey(node.map[0][0], node.map[0][1] + 1)
        new_map[0] = (node.map[0][0], node.map[0][1] + 1)
        new_map[swapping_key] = current_index_of_zero
        immutable_dict_key = tuple(sorted(new_map.items()))

        if config.get(immutable_dict_key) is None:
            config[immutable_dict_key] = 1
            newNode = Node(node)
            newNode.addMap(new_map)
            newNode.addIndex(node.getIndex() + 1)
            return newNode, True
        else:
            return node, False

def ActionMoveLeft(node):
    """
    Moves the empty space left in the puzzle.

    Parameters:
    - node: The current node.

    Returns:
    - The new node after moving left.
    - True if it is possible to move left, False otherwise.
    """
    if node.map[0][1] == 0:
        return node, False
    else:
        new_map = node.map.copy()
        current_index_of_zero = (node.map[0][0], node.map[0][1])
        
        swapping_key = node.getKey(node.map[0][0], node.map[0][1] - 1)
        new_map[0] = (node.map[0][0], node.map[0][1] - 1)
        new_map[swapping_key] = current_index_of_zero

        immutable_dict_key = tuple(sorted(new_map.items()))
        if config.get(immutable_dict_key) is None:
            config[immutable_dict_key] = 1
            newNode = Node(node)
            newNode.addMap(new_map)
            newNode.addIndex(node.getIndex() + 1)
            return newNode, True
        else:
            return node, False
        
def map_values(state):
    """
    Maps the values of the puzzle state.

    Parameters:
    - state: The puzzle state.

    Returns:
    - A dictionary representing the mapped values.
    """
    map = {}
    for i in range(len(state)):
        for j in range(len(state[i])):
            value = state[i][j]
            map[value] = (i, j)
    return map

def solvePuzzle(init_state, goal_state):
    """
    Solves the puzzle using breadth-first search.

    Parameters:
    - init_state: The initial state of the puzzle.
    - goal_state: The goal state of the puzzle.
    """
    node = Node()
    node.addIndex(0)
    inital_node = node
    node.map_indexes(init_state)
    first_config = tuple(sorted(node.map.items()))
    config[first_config] = 1

    with open("NodesInfo.txt", "w") as file:
        file.write("node_Index\tparent_node_index\tnodeInfo\n")
        file.write(str(0) + "\t" + str(0) + "\t" + str(node.map) + "\n")

    queue = deque() 

    while True:
        if node.map == map_values(goal_state):
            print("Goal state reached")
            print(node.map)
            break

        queue.append(node)

        nodeDown, canMoveDown = ActionMoveDown(node)
        nodeUp, canMoveUp = ActionMoveUp(node)
        nodeRight, canMoveRight = ActionMoveRight(node)
        nodeLeft, canMoveLeft = ActionMoveLeft(node)

        if canMoveDown:
            queue.append(nodeDown)
        if canMoveLeft:
            queue.append(nodeLeft)
        if canMoveRight:
            queue.append(nodeRight)
        if canMoveUp:
            queue.append(nodeUp)
        
        node = queue.popleft()
        if node is not None and node.parent is not None:
            with open('NodesInfo.txt', 'a') as file:
                file.write(str(node.getIndex()) + "\t" + str(node.parent.getIndex()) + "\t" + str(node.map.copy()) + "\n")

    backTracking = []
    with open("nodePath.txt", "w") as file:
        file.write("Printing in order \n")
    while True:
        if node.parent == inital_node:
            backTracking.append(node)
            break

        backTracking.append(node)
        # printhingMatrix(node)
        node = node.parent

    for i in range(len(backTracking) - 1, -1, -1):
        printhingMatrix(backTracking[i])
    
    
    print("Backtracking length", len(backTracking))
    print("total nodes visited", len(config))

def printingAllNodes():
    """
    Prints all the nodes in the puzzle.
    """
    with open("Nodes.txt", "w") as file:
        for key in config.keys():
            file.write(str(key) + "\n")

def printhingMatrix(node):
    """
    Prints the matrix representation of the puzzle.

    Parameters:
    - node: The current node.
    """
    dict = node.map
    ans = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    for key, value in dict.items():
        ans[value[0]][value[1]] = key
    print(ans)
    with open('nodePath.txt', 'a') as file:
        file.write(str(ans) + "\n")

if __name__ == "__main__":
    init_mat = np.array([[1, 2, 3], [8, 0, 4], [7, 6, 5]])  
    goal_state = np.array([[2, 3, 1], [8, 0, 4], [7, 6, 5]])

    solvePuzzle(init_mat, goal_state) 
    printingAllNodes()


