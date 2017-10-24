# Project: Kinematics Pick & Place
## Steps to complete the project
1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code.

[//]: # (Image References)
[frames]: ./misc_images/frames.png
[urdf]: ./misc_images/urdf_values.png
[Orientation]: ./misc_images/wrist_orientation.png
[position]: ./misc_images/wrist_position.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

### A. Writeup / README

* Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.
You're reading it!

### B. Kinematic Analysis
* Run the forward_kinematics demo and evaluate the `kr210.urdf.xacro` file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The two images below show the chosen frames according to the DH parameters assignment algorithm, as well as the location of each joint retrieved from the URDF file (the frames center are the joints themselves in this file).

![DH frames][frames]
![URDF values][urdf]

Given the information above, the DH parameters are:

| Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)   |
| ----- | ---------- | ------ | ------ | ---------- |
| 0->1  | 0          | 0      | 0.75   | q1         |
| 1->2  | -pi/2      | 0.35   | 0      | -pi/2 + q2 |
| 2->3  | 0          | 1.25   | 0      | q3         |
| 3->4  | -pi/2      | -0.054 | 1.5    | q4         |
| 4->5  | pi/2       | 0      | 0      | q5         |
| 5->6  | -pi/2      | 0      | 0      | q6         |
| 6->EE | 0          | 0      | 0.303  | 0          |

* Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector (gripper) pose.

To piece of code below shows how the individual transforms were calculated. The global transform is just the multiplication of the individual ones.

```python
def transf_matrix(alpha, a, d, q):
    tm = Matrix([[            cos(q),           -sin(q),          0,                a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                 [                 0,                 0,            0,              1]])
    return tm

# Create individual transformation matrices
T0_1 = transf_matrix(alpha0, a0, d1, q1).subs(s)
T1_2 = transf_matrix(alpha1, a1, d2, q2).subs(s)
T2_3 = transf_matrix(alpha2, a2, d3, q3).subs(s)
T3_4 = transf_matrix(alpha3, a3, d4, q4).subs(s)
T4_5 = transf_matrix(alpha4, a4, d5, q5).subs(s)
T5_6 = transf_matrix(alpha5, a5, d6, q6).subs(s)
T6_7 = transf_matrix(alpha6, a6, d7, q7).subs(s)

# Transform from base link to end effector
T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
```

* Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

One of the trickiest parts of the project was obtaining `theta1`, `theta2` and `theta3`. The next image was helpful to understand how to get these values, but it's important to take into account that the rotation of the first joint affects the projection of side `B`.

![alt text][position]

With the help of the previous image, the position angles can be already obtained:

```python
wx = px - s[d7]*Rrpy_val[0, 2]
wy = py - s[d7]*Rrpy_val[1, 2]
wz = pz - s[d7]*Rrpy_val[2, 2]

# Calculate ARM joint angles using Geometric IK method
wzp = wz - s[d1]
wxyp = sqrt(pow(wx, 2) + pow(wy, 2)) - s[a1]
gamma = atan2(wzp, wxyp)
delta = atan2(abs(s[a3]), s[d4])

C = s[a2]
B = sqrt(pow(wxyp, 2) + pow(wzp, 2))
A = sqrt(pow(s[d4], 2) + pow(abs(s[a3]), 2))
a = acos((pow(B, 2) + pow(C, 2) - pow(A, 2))/(2*B*C))
b = acos((pow(A, 2) + pow(C, 2) - pow(B, 2))/(2*A*C))

theta1 = atan2(wy, wx)
theta2 = pi/2. - a - gamma
theta3 = pi/2. - b - delta
```

With respect to `theta4`, `theta5` and `theta6`, once the the rotation matrix for the wrist was obtained, calculating these angles was trivial.

![alt text][orientation]

The calculation of the orientation given `pitch`, `roll` and `yaw` is shown in the next snippet. Note that it has been corrected with the rotation matrix used in the URDF file.

```python
# Define total rotation matrix
def rot_z(angle):
    return Matrix([[ cos(angle),     -sin(angle),    0 ],
                   [ sin(angle),      cos(angle),    0 ],
                   [    0,             0,            1 ]])
def rot_y(angle):
    return Matrix([[ cos(angle),     0,     sin(angle) ],
                   [     0,          1,          0     ],
                   [-sin(angle),     0,     cos(angle) ]])
def rot_x(angle):
    return Matrix([[ 1,            0,            0       ],
                   [ 0,        cos(angle),   -sin(angle) ],
                   [ 0,        sin(angle),    cos(angle) ]])

R_corr = rot_z(pi) * rot_y(-pi/2.)
Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr # R0_6
```

Using a Jupyter Notebook to make symbolic calculation, `R0_3` was obtained:

```python
R0_3 = (T3_4 * T4_5 * T5_6)[0:3, 0:3]
print(simplify(R0_3))

Matrix([[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
        [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5)],
        [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4), sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6), sin(q4)*sin(q5)]])
```

At this point the three remaining angles were obtained as follows:

```python
theta4 = atan2(R3_6_val[2, 2], -R3_6_val[0, 2])
theta5 = atan2(sqrt(pow(R3_6_val[0, 2], 2) + pow(R3_6_val[2, 2], 2)), R3_6_val[1, 2])
theta6 = atan2(-R3_6_val[1, 1], R3_6_val[1, 0])
```

### C. Project Implementation

* Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.

To improve the execution speed, all the matrices calculations have been done outside the `for` loop and then replaced by each value. Moreover, the inverse matrix have been replaced by the transpose calculation. Since the code had been proven to be quite accurate using the inverse and forward Kinematics in `IK_debug.py`, getting the simulation to work was fairly easy.

The objects are successfully put into the box. In some cases the trajectory doesn't match the expected one because there are multiple solutions for the joint angles. Limiting those results would improve it.
