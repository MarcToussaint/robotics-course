## this should be the default panda we use on the real system
#  with NO dofs for the gripper

Include: '../rai-robotModels/panda/panda.g'

# modify default home pose
Edit panda_joint2 { q= -.5 }
Edit panda_joint4 { q= -2 }


## delete original gripper

Delete panda_hand_joint
Delete panda_hand_1
Delete panda_hand_0
Delete panda_hand>panda_finger_joint1
Delete panda_hand>panda_finger_joint2
Delete panda_finger_joint1
Delete panda_finger_joint2
Delete panda_leftfinger_1
Delete panda_leftfinger_0
Delete panda_rightfinger_1
Delete panda_rightfinger_0

Delete gripper
Delete palm
Delete finger1
Delete finger2

Include: 'gripper.g'

Edit gripper (panda_joint7){ Q:<d(-90 0 1 0) d(135 0 0 1) t(0 0 -.155)> }
       
Edit finger1{ joint:transX Q:<> A:<t(+.07 0 -.05)> limits: [-.05 0.02], contact: -2, shape:capsule, size:[.02, .02] }
Edit finger2{ joint:transX mimic:(finger1) Q:<> A:<d(180 0 0 1) t(+.07 0 -.05)>, limits: [-.05 0.02], contact: -2, shape:capsule, size:[.02, .02]}
        
