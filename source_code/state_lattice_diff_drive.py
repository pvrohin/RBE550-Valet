# import the pygame module
import pygame
from math import atan, cos, sin, sqrt, tan, atan, radians, pi
from typing import Callable, Dict, List, Optional
from numpy import arange,linspace
from queue import PriorityQueue

class skidDrive():
    def __init__(self,x,y,theta,exact=False):
        self.start = [50,50,0]
        self.goal = [350,500,0]

        self.x = x
        self.y = y
        self.theta = theta

        if not exact:
            x, y, theta = self.get_rounded()
            self.x = x
            self.y = y
            self.theta = theta
        self.theta = self.theta % (2*pi)

        self.L = 0.4
        self.r = 0.1
        self.max_velocity = 5

        self.increment = 5

        self.steps_taken = 0

        self.queue = PriorityQueue()

        self.parents = {}

        self.costs = {}
        self.costs[tuple(self.start)] = 0

        self.pixels_per_meter = 35

        self.dot_list = []

        self.obstacles = []
  
    def push(self, state, cost=0):
        self.queue.put((cost, state))

    def pop(self):
        return self.queue.get()[1]

    def __len__(self):
        return len(self.queue.queue)
    
    def get_rounded(self):
        x = round(self.x, 2)
        y = round(self.y, 2)
        theta = round(self.theta, 1)
        return (x, y, theta)
    
    def get_neighbors(self, increment : float,time_increment : float):

        neighbors = []

        ur_ul = []

        ur_ul_max = 20.0

        if increment > 1:
            ur_ul = []
            for i in arange(increment,ur_ul_max+increment,increment):
                ur_ul.append((i,20.0))
                ur_ul.append((20.0,i))

        for v in ur_ul: 
            x = self.x
            y = self.y
            theta = self.theta
            for _ in range(100): 
                u_r = v[0]
                u_l = v[1]                                          
                thetadot = (self.r/self.L) * (u_r-u_l)
                thetadelta = thetadot * time_increment
                theta = theta + thetadelta
                xdot = (self.r/2) * (u_r + u_l) * cos(theta)
                xdelta = xdot * time_increment
                ydot = (self.r/2) * (u_r + u_l) * sin(theta)
                ydelta = ydot * time_increment
                x = x + xdelta
                y = y + ydelta 
            neighbors.append([x,y,theta,u_l,u_r])
        
        return neighbors
    
    def search(self):
        self.create_Obstacle()
        path = None
        self.push(self.start)

        while path==None:
            path = self.step()
        return path,self.dot_list,self.obstacles

    def distance_between(self, other):
        return sqrt((self.x-other[0])**2 + (self.y-other[1])**2 + 100*(self.theta-other[2])**2)

    def goal_check(self, other):
        if other is None:
            return False
        
        distance = self.distance_between(other)
        theta_distance = abs(self.theta - other[2])
        if theta_distance > pi:
            theta_distance = (2*pi) - theta_distance

        return distance <= 2.25 and theta_distance < pi/8
    
    def transition_cost(self, to):
        # Steering / Theta penalty
        theta_difference = abs(self.theta - to[2]) % (2*pi)
        if theta_difference > pi:
            theta_difference = (2*pi) - theta_difference
        theta_penalty = 0 * theta_difference
        
        # Distance Penalty
        distance_penalty = self.distance_between(to)
        cost = distance_penalty + theta_penalty
        return cost
    
    def create_Obstacle(self):
        obstacle_list = []

        # Initializing Color
        color = (255,0,0)
 
        # Drawing Rectangle
        obstacle_coordinate_1 = pygame.Rect((30,400), (150,150))
        obstacle_coordinate_2 = pygame.Rect((500,400), (150,150))
        obstacle_coordinate_3 = pygame.Rect((650,200), (150,150))
        
        obstacle_list.append(obstacle_coordinate_1)
        obstacle_list.append(obstacle_coordinate_2)
        obstacle_list.append(obstacle_coordinate_3)

        self.obstacles = obstacle_list.copy()
    
    def step(self):

        # If the queue is empty, game over - no path exists!
        if len(self.queue.queue) == 0:
            raise Exception("no path to the goal exists")

        # Get the next candidate from our current state
        current = self.pop()

        self.x = current[0]
        self.y = current[1]
        self.theta = current[2]

        x, y, theta = self.get_rounded()
        self.x = x
        self.y = y
        self.theta = theta
        self.theta = self.theta % (2*pi)

        # If our current state is our goal state, we've hit our
        # goal, we're done! Let's build the path...
        if self.goal_check(self.goal):
            path = [current]
            while True:
                current = self.parents[tuple(current)]
                path.insert(0, current)
                #print(path)
                if self.start == current:
                    return path
        
        if self.start == current:
            current_cost = 0
        else:
            current_cost = self.costs[tuple(current)]
        
        # Get each neighbor for the current State and queue them. 
        neighbors = self.get_neighbors(increment=self.increment,time_increment = 0.01)

        for neighbor in neighbors:

            width, height = 800, 800
            # don't go out of bounds.
            if neighbor[0] < 0 or neighbor[1] < 0:
                continue
            if neighbor[0] > width or neighbor[1] > height:
                continue

            obstacles = self.obstacles.copy()

            obstacle = obstacles.pop(0)
            if obstacle.collidepoint(neighbor[0],neighbor[1]):
                continue
            obstacle = obstacles.pop(0)
            if obstacle.collidepoint(neighbor[0],neighbor[1]):
                continue
            obstacle = obstacles.pop(0)
            if obstacle.collidepoint(neighbor[0],neighbor[1]):
                continue

            # If we have already reached this state, we don't
            # need reiterate
    
            if tuple(neighbor) not in self.parents:

                self.parents[tuple(neighbor)] = current

                distance = self.distance_between(self.goal)
                heuristic_cost = 5 * distance

                transition_cost = self.transition_cost(neighbor)

                neighbor_cost = current_cost + transition_cost
                self.costs[tuple(neighbor)] = neighbor_cost

                total_cost = neighbor_cost + heuristic_cost

                self.push(neighbor, total_cost)

                #Draw a dot for its current spot
                pos = (neighbor[0], neighbor[1])
                self.dot_list.append(pos)
    
obj = skidDrive(50,50,0)

path,dot_list,obstacle_list = obj.search()

path_list = []

first = path.pop(0)
second = path.pop(0)

while(True):
    firstxy = (first[0], first[1])
    secondxy = (second[0], second[1])
    path_list.append([firstxy,secondxy])
    first = second
    if len(path) == 0:
        break
    second = path.pop(0)

background_colour = (255,255,255)

screen = pygame.display.set_mode((800,800))
  
pygame.display.set_caption('Valet Skid Drive')
  
screen.fill(background_colour)
  
# Update the display using flip
pygame.display.flip()
  
# Variable to keep our game loop running
running = True
  
# game loop
while running:
# for loop through the event queue  
    for event in pygame.event.get():
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            running = False
        pygame.draw.circle(screen, (255,0,0), (50,50), 3, width=5)
        pygame.draw.circle(screen, (255,0,0), (350,500), 3, width=5)
        while(len(obstacle_list)>0):
            obstacle = obstacle_list.pop(0)
            pygame.draw.rect(screen,(0,0,255),obstacle)
        for dot in dot_list:
            screen.fill((0, 0, 0), (dot, (2, 2)))
        for p in path_list:
            pygame.draw.line(screen, (0, 255, 0), p[0], p[1], width=2)
    pygame.display.update()