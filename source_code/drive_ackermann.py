import pygame
import math
from state_lattice_ackermann import Ackermann,path_list

class Environment:
    def __init__(self,dimensions):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.height = dimensions[0]
        self.width = dimensions[1]
        pygame.display.set_caption("Ackermann Drive Robot")
        self.map = pygame.display.set_mode((self.width,self.height))

class Robot:
    def __init__(self,start,robot_image,width):
        self.convert = 37.952 # convert m/s to pixels
        self.w = width
        self.x = start[0]
        self.y = start[1]
        self.theta = 0
        self.psi = 0
        self.v = 0.01*self.convert
        self.L = 2.8

        self.img = pygame.image.load(robot_image)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center = (self.x,self.y))

    def draw(self,map):
        map.blit(self.rotated,self.rect)

    def move(self,path,event=None):

        self.v = path[4]
        self.psi = path[3]

        for _ in range(100):
            self.theta += (self.v/self.L)*math.tan(self.psi)*dt
            self.x += self.v*math.cos(self.theta)*dt
            self.y += self.v*math.sin(self.theta)*dt
        
        self.rotated = pygame.transform.rotozoom(self.img,-math.degrees(self.theta),1)
        self.rect = self.rotated.get_rect(center=(self.x,self.y))

obj = Ackermann(50,50,0,0)

path,dot_list,obstacle_list = obj.search()

path.pop(0)

pygame.init()

start = (50,50)

dimensions = (800,800)

running = True

dt = 0.009
lasttime = pygame.time.get_ticks()

environment = Environment(dimensions)

obstacle_1 = obstacle_list.pop(0)
obstacle_2 = obstacle_list.pop(0)
obstacle_3 = obstacle_list.pop(0)
        
robot = Robot(start,r"ackermann.png",0.01*3779.52)

while len(path)!=0:
    element = path.pop(0)
    pygame.display.update()
    environment.map.fill(environment.white)
    pygame.draw.circle(environment.map, (255,0,0), (50,50), 3, width=5)
    pygame.draw.circle(environment.map, (255,0,0), (350,500), 3, width=5)
    pygame.draw.rect(environment.map,(0,0,255),obstacle_1)
    pygame.draw.rect(environment.map,(0,0,255),obstacle_2)
    pygame.draw.rect(environment.map,(0,0,255),obstacle_3)
    for dot in dot_list:
        environment.map.fill((0, 0, 0), (dot, (2, 2)))
    for p in path_list:
        pygame.draw.line(environment.map, (0, 255, 0), p[0], p[1], width=2)
    robot.move(element)

    robot.draw(environment.map)

