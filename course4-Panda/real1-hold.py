#!/usr/bin/env python
# coding: utf-8

# # First time with real robot
# This starts exactly as script3-BotOp, only switching from BotOp(C, useRealRobot=True) instead of False.

# In[ ]:


from robotic import ry
import numpy as np
import time


# In[ ]:


#in case you switch to simulation
ry.params_add({'botsim/verbose': 1., 'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()


# In[ ]:


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
pcl = C.addFrame('pcl', 'cameraWrist')
C.view(False, 'this is your workspace data structure C -- NOT THE SIMULTATION')


# In[ ]:


# True = real robot!!
bot = ry.BotOp(C, True)


# In[ ]:


bot.hold(True, False)


# In[ ]:


while bot.getKeyPressed()!=ord('q'):
    image, depth, points = bot.getImageDepthPcl("cameraWrist")
    pcl.setPointCloud(points, image)
    bot.sync(C, .1)


# In[ ]:


del bot


# In[ ]:




