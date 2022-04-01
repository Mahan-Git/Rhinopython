"""
Created on Tue Oct 25 13:05:18 2021

@author: Mahan.M
"""

import Rhino
import scriptcontext as sc
import rhinoscriptsyntax as rs
import math
import random as rnd
import copy


class Agent2D(): 
    def __init__(self, position):
        """
        initialize agent with list of position [x, y]
        """
        self.position = position
        self.x = position[0]
        self.y = position[1]
        self.count = None

        self.tol = 1
        self.RhinoPt = None
        self.circle = None
        self.direction = None
        self.neighborPosition = None
        self.neighborPosition1 = None
        self.neighborDistance = None
        self.state = "move"
        self.edge1 = None
    def __repr__(self):
        return "Class Agent2D _ position: x={},y={}".format(self.x, self.y)
    def rnd_move(self):
        vec = rs.CreateVector([rnd.uniform(-1,1),rnd.uniform(-1,1),0])
        self.direction = rs.VectorUnitize(vec)

    def shortDistanceAgent2Agents(self, a_s,o_s):
        """
        input: 
                self position 
                list of neighbors agents to calculate the distance from
        return: 
                distance from closest neighbor agent 
                position of closest neighbor
        """
        list_dist = []
        for a2 in a_s:
            d = self.distance(a2)
            list_dist.append(d)
    
        self.count = 0
        obstacle = True
        while obstacle:

            if (((list_dist.index(min(list_dist))+self.count+1) < len(a_s)) and ((list_dist.index(min(list_dist))-self.count-1) > 0)):
                self.neighborPosition = a_s[list_dist.index(min(list_dist))+self.count] 
                self.neighborPosition1 = a_s[list_dist.index(min(list_dist))-self.count] 
                obstacle = self.checkObstacles(o_s)
                self.neighborDistance = self.distance(self.neighborPosition)  
                #if (No_obstacle == False): 
                #self.edge =  a_s[list_dist.index(min(list_dist))+self.count-1]
                #rs.AddPoint(self.edge1.x,self.edge1.y,0)
                #self.neighborPosition = self.edge1
            else:
                self.neighborPosition = a_s[list_dist.index(min(list_dist))]
                self.neighborDistance = self.distance(self.neighborPosition) 

        
        return min(list_dist), a_s[list_dist.index(min(list_dist))]
    def distance(self, a2):
        """
        check the distance between agent and other agent
        input:
                self position
                other agent instance
        return:
                distance (float)
        """
        return math.sqrt(((self.x-a2.x)**2)+((self.y-a2.y)**2))
    def checkObstacles(self, o_s):

        path = rs.AddLine([self.x,self.y,0],[self.neighborPosition.x,self.neighborPosition.y,0])
        path1 = rs.AddLine([self.x,self.y,0],[self.neighborPosition1.x,self.neighborPosition1.y,0])
        int_pt = None
        int_pt1 = None
        for obs in o_s:
            if (int_pt == None):
                int_pt = rs.CurveCurveIntersection(path, obs)
        for obs in o_s:
            if (int_pt1 == None):
                int_pt1 = rs.CurveCurveIntersection(path1, obs)
         
        if ( int_pt ==None):
            rs.DeleteObject(path)
            rs.DeleteObject(path1)
            return False
            
        elif(int_pt1 ==None):
            rs.DeleteObject(path)
            rs.DeleteObject(path1)
            self.neighborPosition = self.neighborPosition1
            return False
        elif (self.count > 50 ):
            rs.DeleteObject(path)
            rs.DeleteObject(path1)
            return False
        else:
            rs.DeleteObject(path)
            rs.DeleteObject(path1)
            self.count = self.count +1
            return True
    def calculateDirection(self):
        """
        to run: 
                fuction needs to have an initial neighbor as target
        output: 
                update self.direction with normalise vector
        """
        
        vector_unit = [(self.neighborPosition.x - self.x) / self.neighborDistance , 
                    (self.neighborPosition.y - self.y) / self.neighborDistance ,0] #+ vec_vision
        #vector_unit = vector/(math.sqrt(((vector[0])**2)+((vector[1])**2)))
        self.direction = vector_unit

        if self.neighborDistance > self.tol:
            self.state = "move"
        else:
            self.state = "stop"
    def vision(self, o_s):
        vis_list = []
        vision_length = 5
        point2 = [self.x+(self.direction[0]*vision_length),self.y+(self.direction[1]*vision_length),0]
        vis = rs.AddLine([self.x,self.y,0],point2)
        int_list = []
        for i in range( 11):
            vis = rs.RotateObject(vis,[self.x,self.y,0],(30),None,False)     
            vis_list.append(vis)
            
            int_pt = None
            int_pt_list = []
            for obs in o_s:
                int_pt = rs.CurveCurveIntersection(vis, obs)
                if (int_pt != None):
                    int_pt_list.append(int_pt)
            int_pt_list_dist = []
            if (len(int_pt_list)!= 0):
                for int1 in int_pt_list:
                    int_pt_list_dist.append(math.sqrt(((self.x-int1[0][1][0])**2)+((self.y-int1[0][1][1])**2)))
                int_pt = int_pt_list[int_pt_list_dist.index(min(int_pt_list_dist))]
            if (int_pt != None):
                int_list.append(int_pt)
        rs.DeleteObjects(vis_list)
        rs.DeleteObject(vis)
        if (len(int_list)!= 0):
            add_vec = [0,0,0]
            for int in int_list:
                add_vec = [add_vec[0] +(1/(self.x - int[0][1][0])),add_vec[1]+(1/(self.y-int[0][1][1])),0]
            
            add_vec = rs.CreateVector(add_vec)
            add_vec = rs.VectorUnitize(add_vec)
            new_dir = rs.CreateVector([self.direction[0]+ add_vec[0],self.direction[1]+ add_vec[1],0])
            new_dir = rs.VectorUnitize(new_dir)
            self.direction = new_dir
        
        #for obs in o_s:
            #if (int_pt == None):
                #int_pt = rs.CurveCurveIntersection(path, obs)
            #vec_vision = 
 
    def alignment(self,agents):
        R = 20
        new_vec = rs.CreateVector(0,0,0)
        for a in agents:
            a_dist = self.distance(a)
            if (a_dist < R and a_dist != 0 ):
                new_v =  rs.CreateVector(self.direction[0] + a.direction[0],self.direction[1] + a.direction[1],0)
                new_vec = new_vec + new_v 
        self.direction = self.direction + new_vec
        rs.VectorUnitize(self.direction)
    def cohesion(self,agents):
        R = 50
        steering =rs.CreateVector([0,0,0])
        count = 0 
        for a in agents:
            a_dist = self.distance(a)
            if (a_dist < R and a_dist != 0 ):
                count += 1
                average = rs.CreateVector([a.x-self.x, a.y-self.y,0])
                steering = steering + average
        if (count > 0):
            steering = steering/count
            new_dir = steering + self.direction
            self.direction = rs.VectorUnitize(new_dir)
    def separation(self,agents):
        R = 6
        sep_vec = rs.CreateVector([0,0,0])
        for a in agents:
            a_dist = self.distance(a)
            if (a_dist < R and a_dist != 0 ):
                sep_vec += rs.CreateVector([a.x-self.x, a.y-self.y,0])
        self.direction = rs.VectorUnitize((sep_vec*-1) + self.direction)
    def boundary(self,b):
        i=1
        c = True
        while (i < len(b)-1 and c ):
            dist = self.distance(b[i])
            if (dist < 3):
                if (b[i].x == b[i-1].x or b[i].x == b[i+1].x):
                    self.direction = rs.CreateVector([self.direction[0],(self.direction[1])*-2,0])
                elif (b[i].y == b[i-1].y or b[i].y == b[i+1].y):
                    self.direction = rs.CreateVector([(self.direction[0])*-2,self.direction[1],0])
                else:
                    self.direction = rs.CreateVector([(self.direction[0])*-2,(self.direction[1])*-2,0])
                c = False
            i += 1

    def move(self):
        if self.state == "move":
            
            rs.DeleteObject(self.RhinoPt)
            rs.DeleteObject(self.circle)
            self.x += self.direction[0]
            self.y += self.direction[1]
    def drawRhinoPoint(self):
        self.RhinoPt = rs.AddPoint(self.x, self.y)
        self.circle = rs.AddCircle(self.RhinoPt, 3)
        

class CurveAgent2D():
    
    def __init__(self, Xpositions, curveFunc):
        """
        initialize curve agent with list of x position & curve function (python function)
        """
        self.Xpositions = Xpositions
        self.curveFunc = curveFunc
        self.agents = self.curvePts()
        self.curve = None
    
    def __repr__(self):
        return "Class CurveAgent2D"

    def curvePts(self):
        """
        input: 
                x position from a point in the curve
        output: 
                y position referent to x position on the curve
        """
        crvAgents = []
        for x in self.Xpositions:
            y = self.curveFunc(x)
            a = Agent2D([x,y])
            crvAgents.append(a)
        return crvAgents
    
    def drawRhinoCurve(self, type_curve="polyline"):
        """
        input: 
                list of X positions
                type_curve (optinal):   default => polyline
                                        1       => interpolate
        Rhino: 
                Rhino polyline or Rhino CurveInterpolation
        return: 
                array [python list of points [x,y], Rhino points] 
        """
        ERROR = TypeError("Invalid curve type")

        list_pt = []
        for a in self.agents:
            list_pt.append([a.x, a.y]) 

        if type_curve == "polyline":
            rs.AddPolyline(list_pt)
        elif type_curve == 1:
            rs.AddInterpCurve(list_pt)
        else: 
            raise ERROR

class ObstacleAgent2D():
    def __init__(self):
        self.agents = rs.GetObjects()

    def move(self):
        for a in self.agents:
            rs.MoveObject(a, [-.2,rnd.uniform(-.2,.2),0])



#############
### TIMER ###
#############
def TimeConsumingTask():    
        # Was escape key pressed?
        if (sc.escape_test(False)):
            print ("TimeConsumingTask cancelled.")
            return False
        else :
            return True

######################
### INIT VARIABLES ###
######################
x = range(-1000,1001)
def expr(x):
    return (0.5)*x**2 + 10

if __name__=="__main__":
    #cleaning document
####sc.doc.Objects.Clear()

    ###################
    ### INIT AGENTS ###
    ###################
    n = 10
    a_agents = []
    point= rs.GetPoints("select points")
    n = len(point)
    for i in range(n):
        
        #point = rs.GetPoint("a "+ str(i))
        a = Agent2D([point[i].X,point[i].Y])
        a_agents.append(a)
        a.rnd_move()
        a.drawRhinoPoint()
    boundary = rs.GetObject("boundary")
    list = rs.DivideCurve(boundary,1000,False,True)
    b_agents = []
    for b_pt in list:
        b = Agent2D([b_pt[0],b_pt[1]])
        b_agents.append(b)

    #c1 = CurveAgent2D(x, expr)
    #c1.drawRhinoCurve()
    #print(c1.agents)
    #d1 = ObstacleAgent2D()


    ##################
    ### SIMULATION ###
    ##################


    counter = 0
    button = True
    while button:

        for a in a_agents:
            
            #a.vision(d1.agents)
            a.alignment(a_agents)
            a.cohesion(a_agents)
            a.separation(a_agents)
            a.boundary(b_agents)
            a.move()
            a.drawRhinoPoint() 

        # control timer and simulation
        counter += 1
        button = TimeConsumingTask()
        if counter == 10000: #or a.state == "stop":
            break
