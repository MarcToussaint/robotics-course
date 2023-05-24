#!/usr/bin/env python
# coding: utf-8

# In[ ]:


from robotic import ry


# In[ ]:


#import sys, os
#sys.path.append(os.path.expanduser('~/git/rai-python/build'))
#import libry as ry


# In[ ]:


C = ry.Config()
C.addFile('mini.g')
C.view()


# In[ ]:


C.watchFile('mini.g')


# In[ ]:


q = C.getJointState()
print(q)


# In[ ]:


q[0] = q[0] + .5
C.setJointState(q)
C.view()


# In[ ]:


frameC = C.frame('C')
print('pos:', frameC.getPosition(), 'quat:', frameC.getQuaternion())


# In[ ]:


q[0] = q[0] + .5
C.setJointState(q)
print('pos:', frameC.getPosition(), 'quat:', frameC.getQuaternion())


# In[ ]:


[y,J] = C.eval(ry.FS.position, ['C'])
print('position of C:', y, '\nJacobian:', J)
type(J)


# In[ ]:


#only the z-position relative to target 0.5:
C.eval(ry.FS.position, ['C'], [[0,0,1]], [0,0,0.5]) #the scaling is a 1x3 matrix


# In[ ]:


help(ry.FS)


# In[ ]:


del C


# In[ ]:




