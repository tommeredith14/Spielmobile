import sys, pygame, time, math
from random import randint
print("hello")
from pygame.locals import *
pygame.init()
'''
size = width, height = 500, 500
speed = [2, 2]
white = 255, 255, 255
black = 0, 0, 0
screen = pygame.display.set_mode(size)
'''
white = 255, 255, 255
black = 0, 0, 0
WHEELDIST = 10.0
class robotLocation:
    def __init__(self):
        self.x = 100;
        self.y = 100;
        self.angle = 0.0;
        return
    def updatePosition(self, leftDistanceInt,rightDistanceInt):
        leftDistance = float(leftDistanceInt)
        rightDistance = float(rightDistanceInt)
        if leftDistance == rightDistance:
            self.x += (float(rightDistance) * math.sin(self.angle))
            self.y += (float(rightDistance) * math.cos(self.angle))
            return
        if leftDistance > rightDistance:
            radius = leftDistance * WHEELDIST / float(leftDistance-rightDistance)
            theta = float(leftDistance)/radius
            midRadius = radius - WHEELDIST / 2
            xTurnCenter = self.x + midRadius * math.cos(self.angle)
            yTurnCenter = self.y - midRadius * math.sin(self.angle)
            xNew = xTurnCenter - midRadius * math.cos(self.angle+theta)
            yNew = yTurnCenter + midRadius * math.sin(self.angle+theta)
            self.x = xNew
            self.y = yNew
            self.angle += theta
            return
        if rightDistance > leftDistance:
            radius = rightDistance * WHEELDIST / float(rightDistance - leftDistance)
            theta = float(rightDistance)/radius
            midRadius = radius - WHEELDIST / 2
            xTurnCenter = self.x - midRadius * math.cos(self.angle)
            yTurnCenter = self.y + midRadius * math.sin(self.angle)
            xNew = xTurnCenter + midRadius * math.cos(self.angle - theta)
            yNew = yTurnCenter - midRadius * math.sin(self.angle - theta)
            self.x = xNew
            self.y = yNew
            self.angle -= theta
            return
class GridSection:
    def __init__(self):
        self.occupied = False
        return
    def __init__(self,paramOccupied):
        self.occupied = paramOccupied
        return
    def setOccupied(self, paramOccupied):
        self.occupied = paramOccupied
        return
    def getOccupied(self):
        return self.occupied


class Map:

    def __init__(self,paramWidth, paramLength):
        self.length = paramLength
        self.width = paramWidth
        self.grid = [[GridSection(False) for x in range (paramLength)] for y in range (paramWidth)]
        return

def DrawRobot(robot):
    pygame.draw.circle(screen,black,(int(robot.x*2), int(robot.y*2)),5)
    screen.set_at((int(2*robot.x+15*math.sin(robot.angle)),int(2*robot.y+15* math.cos(robot.angle))),black)
    return

def DrawMap(map):
    for x in range (0,map.width):
        for y in range (0,map.length):
            if map.grid[x][y].occupied:
                pygame.draw.rect(screen,black,( 2*x, 2*y,1000/500, 1000/500))
            else:
                pygame.draw.rect(screen,white,( 2*x, 2*y, 1000/500, 1000/500))
    return



print("hello")

map = Map(250,250)
robot = robotLocation()
'''
print ("hello")
c = 0
while c < 1000:
    print("hello")
    occux = randint(0,249)
    occuy = randint(0,249)
    map.grid[occux][occuy].setOccupied(True)
    screen.fill(white)
    DrawMap(map)
    pygame.display.flip()
    time.sleep(0.5)
    c += 1

'''
APPLICATION_x_size = 500
APPLICATION_y_size = 500
screen = pygame.display.set_mode((APPLICATION_x_size, APPLICATION_y_size))
pygame.display.set_caption('Fun Boring Example comes with Source Code too!!')
pygame.mouse.set_visible(True)
#pygame.mouse.set_visible(False)
black_square_that_is_the_size_of_the_screen = pygame.Surface(screen.get_size())
black_square_that_is_the_size_of_the_screen.fill((0, 0, 0))
screen.blit(black_square_that_is_the_size_of_the_screen, (0, 0))
pygame.display.flip()
Weeee = True
while Weeee:
    # a color can be: (0 to 255, 0 to 255, 0 to 255)
    My_red_color = (255, 0, 0)
    My_blue_color = (0, 0, 255)
    My_green_color = (0, 255, 0)
    My_yellow_color = (255, 255, 0)
    WHITE_WHITE_HOORAY = (255, 255, 255)
    My_light_red_color = (255, 180, 180)
    My_light_blue_color = (190, 190, 255)

    '''

    '''
    occux = randint(0,249)
    occuy = randint(0,249)
    #map.grid[occux][occuy].setOccupied(True)
    leftDist = -4#randint(0,5)#input("Enter dist of left wheel")
    rightDist = 5#randint(0,5)#input("Enter dist of right wheel")
    robot.updatePosition(leftDist,rightDist)
    screen.fill(white)
    DrawMap(map)
    DrawRobot(robot)
    # If you delete the below line you should no longer see the vibrant colors.
    pygame.display.flip()
    # if the 'X' button is pressed the window should close:
    Geesh = pygame.event.get()
    if len(Geesh) > 0:
     if Geesh[0].type == QUIT: Weeee = False
## Once this line is reached the window should close