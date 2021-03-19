# -*- coding: utf-8 -*-
"""
ASEN 5519
ALGORITHMIC MOTION PLANNING

VECTOR FIELD DISPLAY

Program to fdispaly the potential vector field caused by goal and obstacles

input: obstacles, goal, start
output: visual display of vector field

@author: shrivatsan
"""
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
from matplotlib.colors import ListedColormap
from shapely.geometry import box
from shapely.ops import nearest_points


class Obstacle: #defining all obstacles as objects of this class
        
    def __init__(self, n ):
       
        self.no_of_vertices = n   # user specified number of vertices for an obstacle
        self.vertices = []     # Create a list to store all vertices
       
    
    def create_obstacle(self):   # method to create the obstacle from its vertices
        
        print("\n\nenter coordinates of vertices in the following format : if vertex is (2,3)")
        print("then enter 2,3 i.e type 2 followed by a comma followed 3")
        for i in range(self.no_of_vertices):
            self.vertices.append(tuple(float(x) for x in input('enter {} vertex: '.format(i+1)).split(','))) #create a list which stores all vertices as tuples
        self.obstacle = Polygon(self.vertices) #create a Polygon object using shapely module
        
    def display_obstacle(self):
        
        obstacle_x, obstacle_y = self.obstacle.exterior.xy  #this method is defined in class Polygon
        plt.fill(obstacle_x, obstacle_y)
        
    def is_in_obstacle(self, line):
        return line.intersects(self.obstacle)   # binary predicate defined in shapely module



def distance(p1,p2):
   """finds distance between p1 and p2""" 
   return np.sqrt(np.sum(np.power(p2-p1,2)))

def find_point_c(point,poly):
    "returns the nearest point on polygon from the a point outside the polygon"
     return(nearest_points(poly, point)[0])



def gradient_u_attractive(p1,p2,d_star):
    "calculate the attractive gradient at a point due to goal"
    grad_at_x = 0.0
    grad_at_y = 0.0
    
    if distance(p1,p2) < d_star:            #if distance is less than d star, chose a quadratic potential function
        grad_at_x = 0.1*(p1[0]-p2[0])
        grad_at_y = 0.2*(p1[1]-p2[1])
        
        return (grad_at_x, grad_at_y)  
    else:                                     # else the potential function is linear
        grad_at_x = 0.2*d_star*(p1[0]-p2[0])/distance(p1,p2)
        grad_at_y = 0.1*d_star*(p1[1]-p2[1])/distance(p1,p2)
        
        return (grad_at_x, grad_at_y)  

def gradient_u_repulsive(p1,q_star):
        "calculate the repulsive gradient at a point due to obstacles"

    grad_re_x = 0.0
    grad_re_y = 0.0
    
    robot_x, robot_y = p1
    robot_pos = Point(robot_x, robot_y)
    
    for o in obstacles:
       
        if robot_pos.distance(o) <= q_star:  #if within the q_star range, repuslive gradient is inversely proportional to distance from obstacle
            c = find_point_c(robot_pos ,o)
            c = np.array([c.x,c.y])
            
            grad_dist_x = p1[0]-c[0]/distance(c,p1)
            grad_dist_y = p1[1]-c[1]/distance(c,p1)
            
            grad_re_obs_x = grad_dist_x*0.1*((1/q_star) - (1/robot_pos.distance(o)))*(1/(robot_pos.distance(o)**2))
            grad_re_obs_y = grad_dist_y*0.1*((1/q_star) - (1/robot_pos.distance(o)))*(1/(robot_pos.distance(o)**2))
            
        elif robot_pos.distance(o) > q_star:    #if the robot is at a distance which is greater than q_star, than the obstacle causes no repulsion
             grad_re_obs_x = 0.0
             grad_re_obs_y = 0.0
        
        grad_re_x = grad_re_x + grad_re_obs_x
        grad_re_y = grad_re_y + grad_re_obs_y
              
           
            
    return (grad_re_x, grad_re_y)

def gradient_u(p1,p2,d_star,q_star):
    
    grad_x = 0.0
    grad_y = 0.0
    
    (grad_at_x, grad_at_y) = gradient_u_attractive(p1,p2,d_star)
    (grad_re_x, grad_re_y) = gradient_u_repulsive(p1,q_star)

    grad_x = grad_at_x + grad_re_x          #gradient is the sum of attractive and repulsive gradients
    grad_y = grad_at_y + grad_re_y
    
    return (grad_x, grad_y) 

d_star = 12     #specify value of d-star here
q_star = 3      #specify value of q-star here

start_point = np.array([0.0,0.0])       #specify start point here
goal = np.array([10.0,10.0])            #specify goal point here    
p2 = goal

# Grid of x, y points
nx, ny = 10, 10
x = np.linspace(-1, 12, nx)
y = np.linspace(-2, 12, ny)
X, Y = np.meshgrid(x, y)
flag = 0
U = []
V = []

for x_pt in x:
        for y_pt in y:
                flag = 0
                point = Point(x_pt, y_pt)
                for o in obstacles:
                    if not o.disjoint(point):       #assigning the gradient to be 0 for points on and inside the obstacle
                        flag = 1
                        grad_x = 0
                        grad_y = 0
                        break
                if flag == 0:                   # assigning components of gradient at each point on the grid
                    p1 = np.array([x_pt,y_pt])
                    grad_x, grad_y = gradient_u(p1,p2,d_star,q_star)
                U.append(-grad_x)
                V.append(-grad_y)

plt.quiver(X,Y,U,V)     #plot the vector field
