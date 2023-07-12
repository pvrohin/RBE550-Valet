import pygame
import math
from state_lattice_trailer import Trailer,path_list

class Environment:
    def __init__(self,dimensions):
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.height = dimensions[0]
        self.width = dimensions[1]
        pygame.display.set_caption("Trailer Truck")
        self.map = pygame.display.set_mode((self.width,self.height))

class Robot:
    def __init__(self,start,truck_image,trailer_image,width):
        self.convert = 37.952 # convert m/s to pixels
        self.w = width
        self.x = start[0]
        self.y = start[1]
        self.theta = 0
        self.psi = 0
        self.trailer_theta = 0
        self.vl = 0.01*self.convert
        self.vr = 0.01*self.convert
        self.v = 0.01*self.convert
        self.L = 2.8
        self.max_speed = 0.02*self.convert
        self.min_speed = 0.02*self.convert

        #truck
        self.img_1 = pygame.image.load(truck_image)
        self.rotated_1 = self.img_1
        self.rect_1 = self.rotated_1.get_rect(center = (self.x,self.y))

        #trailer
        self.img_2 = pygame.image.load(trailer_image)
        self.rotated_2 = self.img_2
        self.rect_2 = self.rotated_2.get_rect(center = (self.x-70,self.y))

        self.rect_car = self.img_1.get_rect()
        self.rect_trail = self.img_2.get_rect()

    def draw(self,map):
        map.blit(self.rotated_1,self.rect_1)
        map.blit(self.rotated_2,self.rect_2)

    def move(self,path,event=None):
        self.v = path[5]
        self.psi = path[3]
        self.trailer_theta = path[4]

        for _ in range(100):
            self.theta += (self.v/self.L)*math.tan(self.psi)*dt
            self.x += self.v*math.cos(self.theta)*dt
            self.y += self.v*math.sin(self.theta)*dt
            self.trailer_theta += (self.v/5)*math.sin(self.theta-self.trailer_theta)*dt

        self.rotated_1 = pygame.transform.rotozoom(self.img_1,-math.degrees(self.theta),1)
        self.rect_1 = self.rotated_1.get_rect(center=(self.x,self.y))

        self.rotated_2 = pygame.transform.rotozoom(self.img_2,-math.degrees(self.trailer_theta),1)
        self.rect_2 = self.rotated_2.get_rect(center=(self.x,self.y))

obj = Trailer(50,50,0,0,0)

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
        

robot = Robot(start,r"ackermann.png",r"trailer.png",0.01*3779.52)

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
