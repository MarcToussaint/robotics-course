import numpy as np
import copy
import math

from draw_object import ConfigureEnv

# python3 -m pip install pyopengl
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

# python3 -m pip install pygame
import pygame
from pygame.locals import *

class CarSimulator:
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.theta = 0.
        self.tau = 1.  # one second stime steps
        self.L = 2.  # 2 meters between the wheels
        self.dynamicsNoise = .03
        self.observationNoise = .5
        self.landmarks = np.random.normal(0., 10., (2, 2))

        self.env = self.init_env()
        self.particlesToDraw = []
        self.gaussianMeansToDraw = []
        self.gaussianCovsToDraw = []

    def init_env(self):
        pygame.init()
 
        display = (1000,750)
        pygame.display.set_mode(display, DOUBLEBUF|OPENGL, RESIZABLE)
        
        glRotatef(-45.,1.,0, 0)
        glScaled(0.03, 0.03, 0.03);

        env = ConfigureEnv()
        return env

    def step(self, u):
        v = u[0]
        phi = u[1]
        self.x += self.tau * v * np.cos(self.theta)
        self.y += self.tau * v * np.sin(self.theta)
        self.theta += self.tau * (v / self.L) * np.tan(phi)

        if self.dynamicsNoise > 0.:
            self.x += self.dynamicsNoise * np.random.normal()
            self.y += self.dynamicsNoise * np.random.normal()
            self.theta += self.dynamicsNoise * np.random.normal()
        self.draw_globjects()

    def getMeanObservationAtState(self, X):
        Y = copy.deepcopy(self.landmarks)
        R = np.array([[np.cos(X[2]), -np.sin(X[2])], [np.sin(X[2]), np.cos(X[2])]])
        p = np.outer(np.ones((len(self.landmarks), 1)), np.array([X[0], X[1]]))
        Y -= p
        Y = np.dot(Y, R)
        return Y.flatten()

    def getRealNoisyObservation(self):
        Y = self.getMeanObservationAtState(np.array([self.x, self.y, self.theta]))
        Y += np.random.normal(0., self.observationNoise, len(Y))
        return Y

    def getObservationJacobianAtState(self, X):
        N = len(self.landmarks)
        dy_dx = np.zeros((2 * N, 3))
        for i in range(N):
            J = np.zeros((2, 3))
            # by x
            J[0, 0] = -np.cos(X[2])
            J[1, 0] = np.sin(X[2])
            # by y
            J[0, 1] = -np.sin(X[2])
            J[1, 1] = -np.cos(X[2])
            # by theta
            J[0, 2] = -np.sin(X[2]) * (self.landmarks[i, 0] - X[0]) + np.cos(X[2]) * (self.landmarks[i, 1] - X[1])
            J[1, 2] = -np.cos(X[2]) * (self.landmarks[i, 0] - X[0]) - np.sin(X[2]) * (self.landmarks[i, 1] - X[1])
            dy_dx[i * 2] = J[0]  # copy in big J
            dy_dx[i * 2 + 1] = J[1]
        return dy_dx

    def draw_globjects(self):
        # events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            self.env.mouseMove(event);
        glClearColor(1.0, 1.0, 1.0, 0.0)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable( GL_BLEND );

        # surface
        glColor4f(0,0,0,0.1)
        self.env.draw_surface(20.,20.,0.5)

        # cart
        glPushMatrix()
        glTranslatef(self.x, self.y, 0.)
        glPushMatrix()
        glRotatef(self.theta*180./math.pi, 0., 0., 1.)
        glTranslatef(1.,0., 0.)

        glColor4f(1.,0.,0.,0.5)
        self.env.draw_cube(3., 1.5, .5)  
        glPopMatrix()
        glPopMatrix()
        
        # landmarks
        for landmark in self.landmarks:
            glPushMatrix()
            glTranslatef(landmark[0], landmark[1], 0.)
            glColor(.2,.8,.2);
            self.env.draw_cylinder(0.1, 1.0)
            glPopMatrix()

        # draw particles
        for particle in self.particlesToDraw:
            glColor4f(0.,0.,1.,0.5);
            self.env.draw_diamond(particle[0], particle[1], 0)

        for i in range(len(self.gaussianMeansToDraw)):
            self.env.draw_Covariance(self.gaussianMeansToDraw[i], self.gaussianCovsToDraw[i])


        # update
        pygame.display.flip()
        pygame.time.wait(10)
