#!/usr/bin/env python
# coding: utf-8

# # 1. Example to grap images from a webcam
# (but this only gives you an rgb, not depth)
# hit 'q' in the window to stop

# In[ ]:


import cv2 as cv


# In[ ]:


cap = cv.VideoCapture(0)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    img = frame
    #img = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # Display the resulting frame
    if len(frame)>0: cv.imshow('img', img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


# In[ ]:


# When everything done, release the capture
cap.release()
cv.destroyAllWindows()


# # 2. Example to convert simulated RGB-D images into point clouds

# In[ ]:


import os
os._exit(0)


# In[ ]:


import cv2 as cv
import numpy as np
from robotic import ry
import time
print(cv.__version__)


# In[ ]:


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))

obj = C.addFrame('obj')
obj.setPose('t(0. 0.1 0.8)')
obj.setShape(ry.ST.ssBox, size=[.05,.05,.05,.005])
obj.setColor([1,.0,0])
obj.setMass(.1)
obj.setContact(True)
C.view()


# In[ ]:


bot = ry.BotOp(C, False)
bot.home(C)


# In[ ]:


fxypxy = bot.getCameraFxypxy("camera")
print(fxypxy)
cameraFrame = C.getFrame("camera")


# In[ ]:


points = []
tau = .01

for t in range(30):
    time.sleep(0.1)

    #grab sensor readings from the simulation
    [rgb, depth] = bot.getImageAndDepth('camera')  #we don't need images with 100Hz, rendering is slow
            
    if len(rgb)>0: cv.imshow('OPENCV - rgb',
                                 cv.cvtColor(rgb, cv.COLOR_BGR2RGB))
    if len(depth)>0: cv.imshow('OPENCV - depth', 0.5* depth)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break
        
    bot.sync(C)


# In[ ]:


cv.destroyAllWindows()
del bot
del C


# # 3. example to use multiple camera attached to different robot frames

# In[ ]:


import os
os._exit(0)


# In[ ]:


from robotic import ry
import numpy as np


# In[ ]:


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
# change the position of the central sensor
f = C.frame("camera")
f.setPosition(f.getPosition()+[0,0,.5])
# add a frame for the additional camera
f = C.addFrame("r_gripperCamera", "r_gripper")
f.setShape(ry.ST.camera, [.1])
f.addAttributes({'focalLength':0.895, 'width':640., 'height':360.})


# In[ ]:


bot = ry.BotOp(C, False)
bot.home(C)


# In[ ]:


img, _ = bot.getImageAndDepth('camera')


# In[ ]:


img, _ = bot.getImageAndDepth('r_gripperCamera')


# In[ ]:


del bot
del C


# In[ ]:




