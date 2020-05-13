#!/usr/bin/env python
# coding: utf-8

# ## push test

# In[1]:


import sys
sys.path.append('../../build')
import libry as ry
import numpy as np
import time


# In[2]:


C = ry.Config()
D = C.view()
C.addFile("model.g")
Xstart = C.getFrameState()


# In[3]:


S = C.simulation(ry.SimulatorEngine.physx, True)


# In[4]:


for k in range (1):
    #restart from same state multiple times
    S.setState(Xstart, np.zeros((51,6)))

    tau = .01
    box = C.frame("box")

    for t in range(300):
        S.step(np.zeros(8), tau)

        if t%10 == 0:
            S.getImageAndDepth()  #we don't need images with 100Hz, rendering is slow

        time.sleep(0.01)

        #some good old fashioned IK
        q = C.getJointState();
        [y,J] = C.evalFeature(ry.FS.positionDiff, ["gripper", "ring"])
        y = y * .005 / np.linalg.norm(y)
        q = q - J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ y
        C.setJointState(q)

        if t%100 == 0:
            p = box.getPosition()
            p[0] += .05
            p[2] += .2
            box.setPosition(p)
            S.setState(C.getFrameState(), np.zeros((51,6)))


# ## grasp test

# In[ ]:


import os
os._exit(0)


# In[1]:


import sys
sys.path.append('../../build')
import libry as ry
import numpy as np
import time


# In[2]:


C = ry.Config()
D = C.view()
C.addFile("model.g")
Xstart = C.getFrameState()
C.selectJoints(["finger1", "finger2"], True)


# In[3]:


S = C.simulation(ry.SimulatorEngine.physx, True)


# In[4]:


tau = .01
S.setState(Xstart, np.zeros((51,6)))

for t in range(900):
    C.computeCollisions()
    
    S.step([], tau, ry.ControlMode.none)

    if t%10 == 0:
        S.getImageAndDepth()  #we don't need images with 100Hz, rendering is slow

    time.sleep(0.01)

    #some good old fashioned IK
    if t<=300:
        q = C.getJointState();
        [y,J] = C.evalFeature(ry.FS.oppose, ["finger1", "finger2", "ring4"])
        y = y * min(.008/np.linalg.norm(y), 1.)
        q = q - J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ y
        C.setJointState(q)
        
    if t==300:
        S.closeGripper("gripper")
        
    if t>300:
        q = C.getJointState()
        [y,J] = C.evalFeature(ry.FS.position, ["gripper"]);
        q = q - J.T @ np.linalg.inv(J@J.T + 1e-2*np.eye(y.shape[0])) @ [0.,0.,-2e-4]
        C.setJointState(q)

    if t==600:
        S.openGripper("gripper")


# In[5]:


S=0
D=0
C=0


# In[ ]:




