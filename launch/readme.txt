This parameter server contains all the variables need to configure the bioloid. it is designed so that you put the bioloid in it's "home position" then run the bioloid_interface program, turn the torque off and then read the raw_state and copy the joint positions into the joint_encoder_offset.  You can also set limits to the joint angles, and change the joint name to servo motor mapping.

To make it eaiser to view this data and modify I have included a spread sheet, the easiest way to get the data from the spread sheet in to the .launch:
1) select all the cells and copy
2) paste into the .launch file
3) the spread sheet will export with tabs and this doesn't work in launch files, so do a find and replace and replace all the tabs with a blank space.

Hope this helps

Any quesiton please contact me:

Brennand Pierce, bren@tum.de
