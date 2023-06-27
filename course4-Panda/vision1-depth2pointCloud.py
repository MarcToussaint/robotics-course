#!/usr/bin/env python
# coding: utf-8

# In[ ]:


#import sys, os
#sys.path.append(os.path.expanduser('~/git/botop/build'))
#import libry as ry
from robotic import ry
import numpy as np
import time
import matplotlib.pyplot as plt


# In[ ]:


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))

obj = C.addFrame('obj')
obj.setPose('t(0. 0.1 0.8)')
obj.setShape(ry.ST.ssBox, size=[.05,.05,.05,.005])
obj.setColor([1,.0,0])
obj.setMass(.1)
obj.setContact(True)
#cameraFrame = C.addFrame("myCamera")
#cameraFrame.setShape(ry.ST.marker, [0.3])
# cameraFrame.setPosition([0,0,2.0])
#cameraFrame.setPosition([0,1.0,2.0])
#cameraFrame.setQuaternion([1,-0.5,0,1])
C.view()


# In[ ]:


bot = ry.BotOp(C, False)
bot.home(C)


# In[ ]:


rgb, depth = bot.getImageAndDepth("camera")

fig = plt.figure(figsize=(10,5))
axs = fig.subplots(1, 2)
axs[0].imshow(rgb)
axs[1].matshow(depth)
plt.show()


# From the lecture slides, we learned
# $$\hat x = f\frac{X}{Z}+p_x,~\hat y = f\frac{Y}{Z}+p_y,$$
# where
# - $\hat x, \hat y$ are image coordinates ($u$-$v$ or pixel),
# - $X, Y, Z$ represent camera coordinates, and
# - $p_x, p_y$ are the image offset.
# 
# Since we know the $Z$ value of each pixel, we can compute their 3D coordinate (in camera frame):
# $$ X = Z\frac{\hat x-p_x}{f},~  Y = Z\frac{\hat y-p_y}{f}.$$

# In[ ]:


fxypxy = bot.getCameraFxypxy("camera")
print(fxypxy)
depth.shape
cameraFrame = C.getFrame("camera")


# In[ ]:


fx, fy = fxypxy[0], fxypxy[1]
px, py = fxypxy[2], fxypxy[3]
R, t = cameraFrame.getRotationMatrix(), cameraFrame.getPosition()
H, W = depth.shape

Ctmp = ry.Config()
points = np.zeros((H,W,3))
#for i in range(H):
if False:
    for j in range(W):
        Z = depth[i,j]
        if Z < 0:
            continue
            
        ## depth is sign-fliped, j: right, i: down
        points[i,j,0] = Z * (j - px) / fx;
        points[i,j,1] = -Z * (i - py) / fy;
        points[i,j,2] = -Z
        
        ## Coordinate transformation (from camera to world) 
        points[i,j] = R@points[i,j] + t
        
        tmp = Ctmp.addFrame("pc"+str(i)+str(j))
        tmp.setShape(ry.ST.sphere, [0.02])
#         tmp.setColor([1,0,0])
        tmp.setColor(rgb[i,j]/255)
        tmp.setPosition(points[i,j])
    Ctmp.view()


# In[ ]:


#C2 = ry.Config()
#pclFrame = C2.addFrame('pcl')
#pclFrame.setPosition(cameraFrame.getPosition())
#pclFrame.setQuaternion(cameraFrame.getQuaternion())
C.delFrame('pcl')
pclFrame = C.addFrame('pcl', 'camera')

rgb, _, points = bot.getImageDepthPcl('camera', False)
pclFrame.setPointCloud(points, rgb)
pclFrame.setColor([1.,0.,0.]) #only to see it when overlaying with truth
C.view_recopyMeshes()
C.view()


# In[ ]:


C.delFrame('pcl')


# In[ ]:


bot.sync(C)


# In[ ]:


del bot
del C


# In[ ]:




