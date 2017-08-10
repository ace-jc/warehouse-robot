#
# === Introduction ===
#
# In this problem, you will build a planner that helps a robot
#   find the best path through a warehouse filled with boxes
#   that it has to pick up and deliver to a dropzone.
# 
# Your file must be called `partA.py` and must have a class
#   called `DeliveryPlanner`.
# This class must have an `__init__` function that takes three 
#   arguments: `self`, `warehouse`, and `todo`.
# The class must also have a function called `plan_delivery` that 
#   takes a single argument, `self`.
#
# === Input Specifications ===
# 
# `warehouse` will be a list of m strings, each with n characters,
#   corresponding to the layout of the warehouse. The warehouse is an
#   m x n grid. warehouse[i][j] corresponds to the spot in the ith row
#   and jth column of the warehouse, where the 0th row is the northern
#   end of the warehouse and the 0th column is the western end.
#
# The characters in each string will be one of the following:
#
# '.' (period) : traversable space. The robot may enter from any adjacent space.
# '#' (hash) : a wall. The robot cannot enter this space.
# '@' (dropzone): the starting point for the robot and the space where all boxes must be delivered.
#   The dropzone may be traversed like a '.' space.
# [0-9a-zA-Z] (any alphanumeric character) : a box. At most one of each alphanumeric character 
#   will be present in the warehouse (meaning there will be at most 62 boxes). A box may not
#   be traversed, but if the robot is adjacent to the box, the robot can pick up the box.
#   Once the box has been removed, the space functions as a '.' space.
# 
# For example, 
#   warehouse = ['1#2',
#                '.#.',
#                '..@']
#   is a 3x3 warehouse. The dropzone is at space (2,2), box '1' is located at space (0,0), 
#   box '2' is located at space (0,2), and there are walls at spaces (0,1) and (1,1). The
#   rest of the warehouse is empty space.
#
# The argument `todo` is a list of alphanumeric characters giving the order in which the 
#   boxes must be delivered to the dropzone. For example, if 
#   todo = ['1','2']
#   is given with the above example `warehouse`, then the robot must first deliver box '1'
#   to the dropzone, and then the robot must deliver box '2' to the dropzone.
#
# === Rules for Movement ===
#
# - Two spaces are considered adjacent if they share an edge or a corner.
# - The robot may move horizontally or vertically at a cost of 2 per move.
# - The robot may move diagonally at a cost of 3 per move.
# - The robot may not move outside the warehouse.
# - The warehouse does not "wrap" around.
# - As described earlier, the robot may pick up a box that is in an adjacent square.
# - The cost to pick up a box is 4, regardless of the direction the box is relative to the robot.
# - While holding a box, the robot may not pick up another box.
# - The robot may put a box down on an adjacent empty space ('.') or the dropzone ('@') at a cost
#   of 1 (regardless of the direction in which the robot puts down the box).
# - If a box is placed on the '@' space, it is considered delivered and is removed from the ware-
#   house.
# - The warehouse will be arranged so that it is always possible for the robot to move to the 
#   next box on the todo list without having to rearrange any other boxes.
#
# An illegal move will incur a cost of 100, and the robot will not move (the standard costs for a 
#   move will not be additionally incurred). Illegal moves include:
# - attempting to move to a nonadjacent, nonexistent, or occupied space
# - attempting to pick up a nonadjacent or nonexistent box
# - attempting to pick up a box while holding one already
# - attempting to put down a box on a nonadjacent, nonexistent, or occupied space
# - attempting to put down a box while not holding one
#
# === Output Specifications ===
#
# `plan_delivery` should return a LIST of moves that minimizes the total cost of completing
#   the task successfully.
# Each move should be a string formatted as follows:
#
# 'move {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot moves
#   to and '{j}' is replaced by the column-coordinate of the space the robot moves to
# 
# 'lift {x}', where '{x}' is replaced by the alphanumeric character of the box being picked up
#
# 'down {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot puts 
#   the box, and '{j}' is replaced by the column-coordinate of the space the robot puts the box
#
# For example, for the values of `warehouse` and `todo` given previously (reproduced below):
#   warehouse = ['1#2',
#                '.#.',
#                '..@']
#   todo = ['1','2']
# `plan_delivery` might return the following:
#   ['move 2 1',
#    'move 1 0',
#    'lift 1',
#    'move 2 1',
#    'down 2 2',
#    'move 1 2',
#    'lift 2',
#    'down 2 2']
#
# === Grading ===
# 
# - Your planner will be graded against a set of test cases, each equally weighted.
# - If your planner returns a list of moves of total cost that is K times the minimum cost of 
#   successfully completing the task, you will receive 1/K of the credit for that test case.
# - Otherwise, you will receive no credit for that test case. This could happen for one of several 
#   reasons including (but not necessarily limited to):
#   - plan_delivery's moves do not deliver the boxes in the correct order.
#   - plan_delivery's output is not a list of strings in the prescribed format.
#   - plan_delivery does not return an output within the prescribed time limit.
#   - Your code raises an exception.
#
# === Additional Info ===
# 
# - You may add additional classes and functions as needed provided they are all in the file `partA.py`.ipynb_checkpoints/
# - Upload partA.py to Project 2 on T-Square in the Assignments section. Do not put it into an 
#   archive with other files.
# - Ask any questions about the directions or specifications on Piazza.
#
import copy
class DeliveryPlanner:

    def __init__(self, warehouse, todo):
        self.warehouse = copy.deepcopy(warehouse)
        self.todo = todo
        self.dropzone = self.find_dropzone(warehouse)
        self.todo_box = self.find_box(warehouse, todo)

    def find_dropzone(self, warehouse):
        for i in range(len(self.warehouse)):
            for j in range(len(self.warehouse[0])):
                if self.warehouse[i][j] == '@':
                    return [i,j]
        #print 'No dropzone is found!'
                
    def find_box(self, warehouse, todo):
        todo_box = []
        for i in range(len(self.todo)):
            for j in range(len(self.warehouse)):
                for k in range(len(self.warehouse[0])):
                    if self.warehouse[j][k] == self.todo[i]:
                        todo_box.append([j,k])
        return todo_box               
        #print 'No box is found!'
    
    def compute_value(self, start):
        
        self.delta = [[-1, 0], # go up
                     [ 0,-1], # go left
                     [ 1, 0], # go down
                     [ 0, 1], # go right
                     [-1,-1], # go up-left
                     [ 1,-1], # go down-left
                     [ 1, 1], # go down-right
                     [-1, 1]] # go up-right
        cost_step = [2, 2, 2, 2, 3, 3, 3, 3]
        
        
        self.value = [[99 for row in range(len(self.warehouse[0]))] for col in range(len(self.warehouse))]
        #self.policy = [[' ' for col in range(len(self.warehouse[0]))] for row in range(len(self.warehouse))]
        self.action = [[0 for row in range(len(self.warehouse[0]))] for col in range(len(self.warehouse))]

        self.value[start[0]][start[1]] = 0
        #self.policy[self.dropzone[0]][self.dropzone[1]] = '@'
        
        change = True

        while change:
            change = False
            
            for x in range(len(self.warehouse)):
                for y in range(len(self.warehouse[0])):
                    if [x,y] != start:

                        for a in range(len(self.delta)):
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]

                            if x2 >= 0 and x2 < len(self.warehouse)\
                            and y2 >= 0 and y2 < len(self.warehouse[0]) and\
                            (self.warehouse[x2][y2] == '.' or [x2,y2] == start or [x2,y2] == self.dropzone):
    
                                v2 = self.value[x2][y2] + cost_step[a]
                                if v2 < self.value[x][y]:
                                    change = True
                                    self.value[x][y] = v2
                                    self.action[x][y] = a
        


        #for i in range(len(self.value)):
        #    print self.value[i]
        #for i in range(len(self.value)):
        #    print self.action[i]

        return self.value
    
    # since the robot cannot get in the grid of box or loadzone when lift or down a box, so
    # check the grids around the target(loadzone or box) and find the grid with lowest cost 
    def find_point(self, target, value):
        temp = []
        for a in range(len(self.delta)):
            x2 = target[0] + self.delta[a][0]
            y2 = target[1] + self.delta[a][1]

            if x2 >= 0 and x2 < len(self.warehouse)\
            and y2 >= 0 and y2 < len(self.warehouse[0]) and\
            (self.warehouse[x2][y2] == '.' or self.warehouse[x2][y2] == '@'):
                temp.append([value[x2][y2], x2, y2])
                
        temp.sort()
        temp.reverse()
        point = temp.pop()
        #print [point[1], point[2]]
        
        return [point[1], point[2]]
        
    # find the path from start to end
    def find_path(self, start, end):
        path = []
        [x,y] = start
        path.append([x, y])
        
        while [x,y] != end:
            x2 = x + self.delta[self.action[x][y]][0]
            y2 = y + self.delta[self.action[x][y]][1]
            x = x2
            y = y2
            path.append([x, y])
        #print path
        return path
    
    def plan_delivery(self):
        cost = 0
        all_moves = []
        start = self.dropzone #initialize the robot position
        value = self.compute_value(start) #get the cost map
        cost_pick = 4
        cost_down = 2
        
        for i in range(len(self.todo_box)):
            end = self.find_point(self.todo_box[i], value) #find the lowest cost gird ajacent to the box
            m, n = end
            
            # find the path_go from loadzone or grid adjacent to loadzone to the grid ajacenet to box
            path_go = self.find_path(end, start)
            path_go.pop()
            path_go.reverse()
            path_go.append('lift')

            cost = cost+value[m][n]+cost_pick # cost on path_go
            
            #print cost
            #After picking up, make grid passable, marked as '.', need to change the entire list elmemnt
            self.warehouse[self.todo_box[i][0]] = str(self.warehouse[self.todo_box[i][0]][:self.todo_box[i][1]])+\
            '.'+str(self.warehouse[self.todo_box[i][0]][self.todo_box[i][1]+1:]) 
            
            # now the robot is adjacent to the box, set this point as start and find end point adjacent to loadzone
            # some path points are repeated due to end-start connection, need to remove them
            start = end
            value = self.compute_value(start) # update the cost map
            end = self.find_point(self.dropzone, value) #find lowest cost point adacent to the loadzone
            m, n = end
            
            # find the path_return from grid adjacent to box to the grid ajacenet to loadzone
            path_return = self.find_path(end, start)
            path_return.pop()
            path_return.reverse()
            path_return.append('down')
            
            cost = cost+value[m][n]+cost_down #cost on path_return
            path_go.extend(path_return)#put the go and return path together for this box
            
            all_moves.extend(path_go)#put the go and return path together for all boxes
            

            #print cost
            #print path_go
            # now the robot is ajacent to the loadzone, set this point as start and update the cost map
            # then go back to the beginning of the loop to find the next box
            start = end
            value = self.compute_value(start)

            
            #for j in range(len(self.warehouse)):
            #    print self.warehouse[j]
            
        #print all_moves

        # based on the all_moves list, do some process to get the corrected output format
        moves = []
        box_num = 1
        for i in range(len(all_moves)):
            if all_moves[i] == 'lift':
                moves.append('lift '+str(box_num))
                box_num += 1
            elif all_moves[i] == 'down':
                moves.append('down '+str(self.dropzone[0])+' '+str(self.dropzone[1]))
            else:
                moves.append('move '+str(all_moves[i][0])+' '+str(all_moves[i][1]))
        '''
        moves = ['move 2 1',
                 'move 1 0',
                 'lift 1',
                 'move 2 1',
                 'down 2 2',
                 'move 1 2',
                 'lift 2',
                 'down 2 2']
        '''
        print cost
        return  moves
