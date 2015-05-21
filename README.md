# HOly
Humanoid Ol (project practical training) at the ICS - Programming a humanoid robot to compete in 3 competitions. (<http://www.ics.ei.tum.de/teaching/ss2015/humanoid-olympics/>)

## Manual Control
Publishing torque and motor commands
```bash
rostopic pub /bioloid_interface/bioloid_msg bioloid_interface/bioloid_msg '{motor_torque: [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]}'
rostopic pub /bioloid_interface/command sensor_msgs/JointState '{header: {stamp: now, frame_id: /world}, name: ['R_SAA', 'L_SAA', 'R_SFE', 'L_SFE', 'R_EB', 'L_EB', 'R_HAA', 'L_HAA', 'R_HR', 'L_HR', 'R_HFE', 'L_HFE', 'R_KFE', 'L_KFE', 'R_AFE', 'L_AFE', 'R_AR', 'L_AR'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}' 
```

## Links
* <https://dxydas.wordpress.com/>
* <http://web.ics.ei.tum.de/~bren/Teaching.php>
* <https://github.com/brennand/bioloid_indigo.git>
* [Autonomous Reinforcement Learning with Experience Replay for Humanoid Gait Optimization](http://www.sciencedirect.com/science/article/pii/S1877050912007375)
* [Explicit analytic solution for inverse kinematics of Bioloid
humanoid robot](http://ieeexplore.ieee.org.eaccess.ub.tum.de/stamp/stamp.jsp?tp=&arnumber=6363315)
* [center of gravity http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=5937193&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D5937193]
