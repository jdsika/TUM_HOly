# HOly

## Setup
1. [ROS](http://www.ros.org/) [installieren](http://wiki.ros.org/indigo/Installation/Ubuntu)
* [MoveIt](http://moveit.ros.org/) installieren
```bash
sudo apt-get install ros-indigo-moveit-full
```
## publish torque/commands
```bash
rostopic pub /bioloid_interface/bioloid_msg bioloid_interface/bioloid_msg '{motor_torque: [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]}'

rostopic pub /bioloid_interface/command sensor_msgs/JoinState '{header: {stamp: now, frame_id: /world}, name: ['R_SAA', 'L_SAA', 'R_SFE', 'L_SFE', 'R_EB', 'L_EB', 'R_HAA', 'L_HAA', 'R_HR', 'L_HR', 'R_HFE', 'L_HFE', 'R_KFE', 'L_KFE', 'R_AFE', 'L_AFE', 'R_AR', 'L_AR'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}' 

```
## Links
* <https://dxydas.wordpress.com/>
* <http://web.ics.ei.tum.de/~bren/Teaching.php>
* <https://github.com/brennand/bioloid_indigo.git>

