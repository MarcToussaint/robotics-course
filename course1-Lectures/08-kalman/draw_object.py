from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import pygame
from pygame.locals import *
import math
import numpy as np

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

        self.square = ((0,1,2,3),
                    (4,5,6,7),
                    (0,4,7,3),
                    (1,5,6,2),
                    (2,6,7,3),
                    (1,5,4,0),)

    def draw_edge_points(self, vertices):
        glBegin( GL_POINTS )
        glColor3f(1,1,1)
        [glVertex3f(self.vertices[i][0], self.vertices[i][1], self.vertices[i][2]) for i in range(8)]
        glEnd()

    def draw_cube_frame(self, x, y, z):
        glLineWidth(3.0)
        glBegin(GL_LINES)
        for i in self.edges:
            glVertex3f(self.vertices[i[0]][0]*(x),self.vertices[i[0]][1]*(y),self.vertices[i[0]][2]*(z));
            glVertex3f(self.vertices[i[1]][0]*(x),self.vertices[i[1]][1]*(y),self.vertices[i[1]][2]*(z));
        glEnd() 

    def draw_surface(self, x, y, z, draw_grid=True):
        glBegin(GL_QUADS);
        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(0.5*x, 0.5*y, -0.5*z)
        glVertex3f(0.5*x, -0.5*y, -0.5*z)
        glVertex3f(-0.5*x, -0.5*y, -0.5*z)
        glVertex3f(-0.5*x, 0.5*y, -0.5*z)
        glEnd()

        if draw_grid:
            x_grid = np.linspace(-0.5*x, 0.5*x, num=int(x))
            y_grid = np.linspace(-0.5*y, 0.5*y, num=int(y))
            glLineWidth(1.0)
            glBegin(GL_LINES)
            for xi, yi in zip(x_grid, y_grid):
                glVertex3f(xi, -0.5*y, -0.5*z)
                glVertex3f(xi, +0.5*y, -0.5*z)
                glVertex3f(0.5*x, yi, -0.5*z)
                glVertex3f(-0.5*x, yi, -0.5*z)
            glEnd()

    def draw_line(self):
        start = [-10., 0., 0.]
        end = [10., 0., 0.]
        glLineWidth(6.0)
        glBegin(GL_LINES)
        glColor3f(0., 0.,1.)
        glVertex3f(start[0],start[1],start[2])
        glVertex3f(end[0],end[1],end[2])
        glEnd() 

    def draw_line2(self):
        start = [0., 0., 0.]
        end = [0., 0., 2.]
        glBegin(GL_LINES)
        glColor3f(0., 1.,0.)
        glVertex3f(start[0],start[1],start[2])
        glVertex3f(end[0],end[1],end[2])
        glEnd() 

    def draw_cylinder(self, radius, length, closed=True):
        style=gluNewQuadric()
        glTranslatef(0, 0, -length/2)
        gluCylinder(style, radius, radius, length, 20, 1)
        if closed:
            glScalef(-1, 1, 1)
            gluDisk(style, 0, radius, 20, 1)
            glTranslatef(0, 0, length)
            glScalef(-1, 1, 1)
            gluDisk(style, 0, radius, 20, 1)
            glTranslatef(0, 0, -length/2)
        else:
            glTranslatef(0, 0, length/2)
        gluDeleteQuadric(style)
        
    def glDrawDiamond(self, x, y, z):

        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(.0, .0, z)
        glVertex3f(x, .0, .0)
        glVertex3f(.0, y, .0)
        glVertex3f(-x, .0, .0)
        glVertex3f(.0, -y, .0)
        glVertex3f(x, .0, .0)
        glEnd()
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(.0, .0, -z)
        glVertex3f(x, .0, .0)
        glVertex3f(.0, -y, .0)
        glVertex3f(-x, .0, .0)
        glVertex3f(.0, y, .0)
        glVertex3f(x, .0, .0)
        glEnd()

    def draw_diamond(self, x, y, z, dx=0.2, dy=0.2, dz=0.2):
        glPushMatrix()
        glTranslated(x, y, z)
        self.glDrawDiamond(dx, dy, dz)
        glPopMatrix()

    def draw_Covariance(self, mean, cov):
        d = np.zeros((101, 2))
        # unit cycle
        for i in range(len(d)):
            phi = 2 * np.pi * float(i) / (len(d) - 1)
            d[i, 0] = np.cos(phi)
            d[i, 1] = np.sin(phi)

        w, V = np.linalg.eig(cov)
        w = np.sqrt(w)

        glBegin(GL_LINE_STRIP)
        glColor4f(0.,0.,1.,0.5)
        for i in range(len(d)):
            d[i] *= w
            d[i] = np.dot(V, d[i])
            d[i] += mean
            glVertex3f(d[i, 0], d[i, 1], 0)
        glEnd()

    def mouseMove(self, event):
        global lastPosX, lastPosY, zoomScale, xRot, yRot, zRot;

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 4: # wheel rolled up
            glScaled(1.05, 1.05, 1.05)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 5: # wheel rolled down
            glScaled(0.95, 0.95, 0.95)

        if event.type == pygame.MOUSEMOTION:
            x, y = event.pos
            dx = x - lastPosX
            dy = y - lastPosY
            mouseState = pygame.mouse.get_pressed();
            if mouseState[0]:

                modelView = (GLfloat * 16)()
                mvm = glGetFloatv(GL_MODELVIEW_MATRIX, modelView)
       
       # To combine x-axis and y-axis rotation
                temp = (GLfloat * 3)();
                temp[0] = modelView[0]*dy + modelView[1]*dx
                temp[1] = modelView[4]*dy + modelView[5]*dx
                temp[2] = modelView[8]*dy + modelView[9]*dx
                norm_xy = math.sqrt(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2])
                glRotatef(math.sqrt(dx*dx+dy*dy), temp[0]/norm_xy, temp[1]/norm_xy, temp[2]/norm_xy)
            lastPosX = x
            lastPosY = y

    def draw_cube(self, x, y, z, draw_frame=True):
        # top
        glBegin(GL_QUADS)
        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(-0.5*x, 0.5*y, 0.5*z)
        glVertex3f(0.5*x, 0.5*y, 0.5*z)
        glVertex3f(0.5*x, 0.5*y, -0.5*z)
        glVertex3f(-0.5*x, 0.5*y, -0.5*z)
        glEnd();
     
        # front
        glBegin(GL_QUADS)
        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(0.5*x, -0.5*y, 0.5*z)
        glVertex3f(0.5*x, 0.5*y, 0.5*z)
        glVertex3f(-0.5*x, 0.5*y, 0.5*z)
        glVertex3f(-0.5*x, -0.5*y, 0.5*z)
        glEnd()
     
        # right
        glBegin(GL_QUADS)
        glNormal3f(1.0, 0.0, 0.0)
        glVertex3f(0.5*x, 0.5*y, -0.5*z)
        glVertex3f(0.5*x, 0.5*y, 0.5*z)
        glVertex3f(0.5*x, -0.5*y, 0.5*z)
        glVertex3f(0.5*x, -0.5*y, -0.5*z)
        glEnd()
     
        # left
        glBegin(GL_QUADS)
        glNormal3f(-1.0, 0.0, 0.0)
        glVertex3f(-0.5*x, -0.5*y, 0.5*z)
        glVertex3f(-0.5*x, 0.5*y, 0.5*z)
        glVertex3f(-0.5*x, 0.5*y, -0.5*z)
        glVertex3f(-0.5*x, -0.5*y, -0.5*z)
        glEnd()
     
        # bottom
        glBegin(GL_QUADS)
        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(0.5*x, -0.5*y, 0.5*z)
        glVertex3f(-0.5*x, -0.5*y, 0.5*z)
        glVertex3f(-0.5*x, -0.5*y, -0.5*z)
        glVertex3f(0.5*x, -0.5*y, -0.5*z)
        glEnd()
     
        # back
        glBegin(GL_QUADS)
        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(0.5*x, 0.5*y, -0.5*z)
        glVertex3f(0.5*x, -0.5*y, -0.5*z)
        glVertex3f(-0.5*x, -0.5*y, -0.5*z)
        glVertex3f(-0.5*x, 0.5*y, -0.5*z)
        glEnd()

        if draw_frame:
            glColor4f(0.6,0.0,0.0,1.) 
            self.draw_cube_frame(x, y, z)
