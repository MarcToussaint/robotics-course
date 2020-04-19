from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import pygame
from pygame.locals import *
import math

lastPosX = 0;
lastPosY = 0;
zoomScale = 1.0;
dataL = 0;
xRot = 0;
yRot = 0;
zRot = 0;

class ConfigureEnv(object):
    def __init__(self):

        self.vertices = ((0.5, -0.5, -0.5),
                     (0.5, 0.5, -0.5),
                     (0.5, 0.5, 0.5),
                     (0.5, -0.5, 0.5),
                     (-0.5, -0.5, -0.5),
                     (-0.5, 0.5, -0.5),
                     (-0.5, 0.5, 0.5),
                     (-0.5, -0.5, 0.5))

        self.edges = ((0,1),
                 (0,3),
                 (0,4),
                 (2,1),
                 (2,3),
                 (2,6),
                 (6,5),
                 (6,7),
                 (4,5),
                 (4,7),
                 (3,7),
                 (1,5))


    def draw_edge_points(self, vertices):
        glBegin( GL_POINTS );
        glColor3f(1,1,1); 
        for i in range(0,8):
             glVertex3f(self.vertices[i][0], self.vertices[i][1], self.vertices[i][2]);
        glEnd()

    def draw_cube(self, x, y, z):
        glBegin(GL_LINES)
        for i in self.edges:
            glVertex3f(self.vertices[i[0]][0]*(x),self.vertices[i[0]][1]*(y),self.vertices[i[0]][2]*(z));
            glVertex3f(self.vertices[i[1]][0]*(x),self.vertices[i[1]][1]*(y),self.vertices[i[1]][2]*(z));
        glEnd() 

    def draw_line(self):

        start = [-10., 0., 0.]
        end = [10., 0., 0.]

        glBegin(GL_LINES)
        glColor3f(0., 0.,1.); 
        glVertex3f(start[0],start[1],start[2]);
        glVertex3f(end[0],end[1],end[2]);
        glEnd() 

    def draw_line2(self):

        start = [0., 0., 0.]
        end = [0., 0., 2.]

        glBegin(GL_LINES)
        glColor3f(0., 1.,0.); 
        glVertex3f(start[0],start[1],start[2]);
        glVertex3f(end[0],end[1],end[2]);
        glEnd() 

    def mouseMove(self, event):
        global lastPosX, lastPosY, zoomScale, xRot, yRot, zRot;
     
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 4: # wheel rolled up
            glScaled(1.05, 1.05, 1.05);
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 5: # wheel rolled down
            glScaled(0.95, 0.95, 0.95);
     
        if event.type == pygame.MOUSEMOTION:
            x, y = event.pos;
            dx = x - lastPosX;
            dy = y - lastPosY;
            
            mouseState = pygame.mouse.get_pressed();
            if mouseState[0]:

                modelView = (GLfloat * 16)()
                mvm = glGetFloatv(GL_MODELVIEW_MATRIX, modelView)
       
       # To combine x-axis and y-axis rotation
                temp = (GLfloat * 3)();
                temp[0] = modelView[0]*dy + modelView[1]*dx;
                temp[1] = modelView[4]*dy + modelView[5]*dx;
                temp[2] = modelView[8]*dy + modelView[9]*dx;
                norm_xy = math.sqrt(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2]);
                glRotatef(math.sqrt(dx*dx+dy*dy), temp[0]/norm_xy, temp[1]/norm_xy, temp[2]/norm_xy);

            lastPosX = x;
            lastPosY = y;