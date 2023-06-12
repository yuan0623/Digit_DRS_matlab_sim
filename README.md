# Foot-Placement Control for Underactuated Bipedal Walking on Swaying Rigid Surfaces Using Digit

1. run main program: `digit_main` to have robot motion trajectory
  - adjust DRS motion: line 48-52, change T_DRS_x, T_DRS_y, amplitude_x, amplitude_x to get the desired DRS motion you want.

2. animation
  - run `robot = importrobot('urdf/digit_model_reduced.urdf')`
  - run `Tool.generateAnimiation(robot,x_sol,t,LIP_para,1,'video_name')``
