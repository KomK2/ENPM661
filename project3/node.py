class Node:
    def __init__(self, x, y,orientation, cost, heuristic  ,parent ):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.cost = cost
        self.parent = parent
        self.heuristic = heuristic
        self.total_cost = self.updateCost()
        # self.updateCost()
        
    
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

    def updateCost(self):
        total_cost = self.cost + self.heuristic
        return total_cost
        # if self.total_cost is None or total_cost < self.total_cost:
        #     self.total_cost = total_cost
    
    
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.orientation == other.orientation
    
    def __hash__ (self):
        return hash((self.x, self.y))
    
    def __lt__(self, other):
        return self.cost < other.cost