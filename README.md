[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project

## Commands to run the code for this project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

---


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/diagram.jpg
[image5]: ./misc_images/theta1.jpg
[image6]: ./misc_images/theta2.png
[image7]: ./misc_images/theta3.png
[image8]: ./misc_images/theta456_1.png
[image9]: ./misc_images/success_pic.png
[image10]: ./misc_images/action1.png
[image11]: ./misc_images/action2.png

---
## Writeup

### Kinematic Analysis

Below is the hand-drawn diagram with values derived from the kr210.urdf.xacro file
![alt text][image4]

Below is the digitized table of my DH parameters:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | 0 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

And here is the Python dictionary version:
```python
DH_matrix = {
    'alpha0': 0,        'r0': 0,        'd1': 0.75,
    'alpha1': -pi / 2., 'r1': 0.35,     'd2': 0,
    'alpha2': 0,        'r2': 1.25,     'd3': 0,
    'alpha3': -pi / 2., 'r3': -0.054,   'd4': 1.50,
    'alpha4': pi / 2.,  'r4': 0,        'd5': 0,
    'alpha5': -pi / 2., 'r5': 0,        'd6': 0,
    'alpha6': 0,        'r6': 0,        'd7': 0.303
}
```

![alt text][image3]

#### Derivation and code implementation
The function for generating each transform matrix is show below:
```python
def get_transform_matrix(alpha, a, d, q):
    TF = numpy.matrix(
        [[cos(q), -sin(q), 0, a],
         [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
         [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha),  cos(alpha) * d],
         [0, 0, 0, 1]])

    return TF

```

##### Derive and calculate theta_1
![alt text][image5]
```python
theta1 = numpy.arctan2(wrist_center[1], wrist_center[0])
```

##### Derive and calculate theta_2
![alt text][image6]
```python
r1 = DH_matrix['r1']
d1 = DH_matrix['d1']
side_a = numpy.sqrt(DH_matrix['r3']**2 + DH_matrix['d4']**2)
side_b = numpy.sqrt(
    (numpy.sqrt(wrist_center[0] ** 2 + wrist_center[1] ** 2) - r1) ** 2 +
    (wrist_center[2] - d1) ** 2)
side_c = DH_matrix['r2']
angle_a = numpy.arccos(max(
    [min([(side_b ** 2 + side_c ** 2 - side_a ** 2) / (2 * side_b * side_c), 1]),
     -1]))
angle_b = numpy.arccos(max(
    [min([(side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c), 1]),
     -1]))
angle_g = numpy.arctan2(
    wrist_center[2] - d1,
    numpy.sqrt(wrist_center[0] ** 2 + wrist_center[1] ** 2) - r1)
theta2 = pi / 2 - angle_a - angle_g
```
Max and min are used with arccos function to avoid numerical errors

##### Derive and calculate theta_3
![alt text][image7]
```python
angle_b = numpy.arccos(max(
    [min([(side_a ** 2 + side_c ** 2 - side_b ** 2) / (2 * side_a * side_c), 1]),
     -1]))
link4_sag = numpy.arctan2(DH_matrix['r3'], DH_matrix['d4'])
theta3 = pi / 2 - (angle_b + link4_sag)
```

##### Derive and calculate theta_4, theta_5, theta_6
![alt text][image8]

There are two different ways to calculate theta, using either the roll (theta4) or yaw (theta6) components. 
I chose to use the roll component to calculate theta5 to keep the convention of calculating joint angles in order from the base out to the final joint.
This calculation of theta5 does not effect the calculation of theta4 or theta6 because both angles are calculated without knowing theta5 apriori. 
```python
T0_1 = get_transform_matrix(DH_matrix['alpha0'], DH_matrix['r0'], DH_matrix['d1'], theta1)
T1_2 = get_transform_matrix(DH_matrix['alpha1'], DH_matrix['r1'], DH_matrix['d2'], theta2 - numpy.pi/2)
T2_3 = get_transform_matrix(DH_matrix['alpha2'], DH_matrix['r2'], DH_matrix['d3'], theta3)
R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
R3_6 = numpy.array(R0_3.transpose() * rotation_end_effector, dtype=float)

theta4 = numpy.arctan2(R3_6[2, 2], -R3_6[0, 2])
theta5 = numpy.arctan2(numpy.sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
theta6 = numpy.arctan2(-R3_6[1, 1], R3_6[1, 0])
```

#####  Finish the rest of the Inverse Kinematics
```python
T3_4 = get_transform_matrix(DH_matrix['alpha3'], DH_matrix['r3'], DH_matrix['d4'], theta4)
T4_5 = get_transform_matrix(DH_matrix['alpha4'], DH_matrix['r4'], DH_matrix['d5'], theta5)
T5_6 = get_transform_matrix(DH_matrix['alpha5'], DH_matrix['r5'], DH_matrix['d6'], theta6)
T6_G = get_transform_matrix(DH_matrix['alpha6'], DH_matrix['r6'], DH_matrix['d7'], 0)
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

R_correct = numpy.array([[0, 0, 1.0, 0], [0, -1.0, 0, 0], [1.0, 0, 0, 0], [0, 0, 0, 1.0]])
# Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_7
# With orientation correction applied
T0_G_corrected = (T0_G * R_correct)

FK = T0_G_corrected
```

#### Conclusion
Here's some actions shots
![alt text][image10]

![alt text][image11]

Here's the final result of 9 objects moved into the bucket.
![alt text][image9]

I used numpy matrix operations to speed up the runtime for handling the inverse kinematics.  The code works well most of the time typically getting 9/10 objects successfully into the drop bucket. 
But it does fail in certain edge cases that I haven't been able to solve for. 
Occasionally the IK_handler function receives 0 valid poses from the motion planner and is unable to pick up the target object, and sometimes the object is dropped during path from the shelf to the bucket.
None of those errors are particularly common though as the success rate is ~90%.  

There was also some issues with the motion planning software.  At each pose during the planned trajectory there is a slight pause and the following warning is output by rviz:

    [warn] Dropping first 1 trajectory point(s) out of N, as they occur before the current time. First valid point will be reached in 0.XXs

With more time, I would first like to find a fix for warnings from the motion planner, which would  fix the choppy movements.  I'd also like to figure out why I sometimes receive a trajectory with zero poses.


