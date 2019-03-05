from Queue import PriorityQueue
from Queue import Queue

#defines a basic node class
class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in, pathcost_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in
        self.pathcost = pathcost_in

    def printme(self):
        print "\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid

#initialize the priority queue
q = PriorityQueue()

#insert objects of form (priority, Node)
q.put((1.46, Node(2,3,0.3,1,0,0)))  #similar to push
q.put((2.6, Node(5,2,0.1,2,1,0)))
q.put((5.6, Node(2,3,0.3,3,2,0)))
q.put((0.6, Node(4,3,0.2,4,1,0)))

print "Pop elements of queue in order of increasing priority:"
while not q.empty():
    next_item = q.get()  # like pop
    print "Priority:", next_item[0]
    next_item[1].printme()
