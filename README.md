**Robotics Nanodegree**\
P2: Robotic Arm: Pick and Place\
Markus Buchholz

**Introduction\
\
**The goal for this project was to created forward and inverse kinematic
model for KUKA KR210 robot arm. The model development, debugging and
testing was performed by use of ROS and Rviz. Developed kinematic model
allows to pick objects from shelf and place them to refuse heap (8/10).
Debug process was performed in the same development environment where
several robot position and joint angles(orientations) were verified.

**Kinematic Analysis**

![](media/image1.png){width="6.531944444444444in" height="3.55in"}

1.  **DH Parameters**

KUKA robot kinematic model can be presented as follows. Following sketch
allowed to defined the DH parameters.

  i   alpha(i-1)   a (i-1)   d (i)   q (i)
  --- ------------ --------- ------- -------------
  1   0            0         0.75    
  2   -pi/2        0.35      0       q2: q2-pi/2
  3   0            1.25      0       
  4   -pi/2        -0.054    1.50    
  5   pi/2         0         0       
  6   -pi/2        0         0       
  7   0            0         0.303   q7: 0

1.  **Transformation Matrices**

Homogenous transform matrix TF\_Matrix is a skeleton in order to compute
individual transform matrices about each joint (using the DH\_table).

![](media/image2.png){width="6.531944444444444in"
height="1.3138888888888889in"}

![](media/image3.png){width="5.3411931321084865in"
height="1.586003937007874in"}

Homogeneous transform matrix T0\_EE from base\_link to gripper\_link can
be given by following matrix equations:

![](media/image4.png){width="4.624429133858268in"
height="1.9842858705161854in"}

1.  **Decouple Inverse Kinematics **

Inverse kinematic for 6 axis robot will be performed by kinematical
decoupling, where in order to solve the whole problem can be divided by
two subtasks: 1. Position inverse kinematic problem and 2. Orientation
inverse kinematic problem.

a.  **Position inverse kinematic problem**

Analyzing following figure is can be estimated that:

![](media/image5.png){width="6.531944444444444in"
height="4.252083333333333in"}

**R0\_6 = R\_EE** (Euler Rotation matrix: roll, pitch, yaw).

**wc = p - dG\*R\*k**

**wc = p - dG\*R\_EE\*k, where k = \[0,0,1\].T**

p = \[Px, Py, Pz\] = end-effector positions

Wc = \[Wx, Wy, Wz\] = wrist positions

In spherical wrist, axes z4, z5 and z6 intersects in the same O point.
Therefore O4, O5 and O6 are placed in the same place in the middle of
wrist. This affects that movement of axis 4, 5 or 6 does not influence
on position of point O, therefore position of O is the function of
theta1, theta2 and theta3.

![](media/image6.png){width="3.7111351706036744in"
height="2.405964566929134in"}

Applying the correction (URDF frame is different):

The ROT\_EE can be given as follows:

![](media/image7.png){width="6.531944444444444in"
height="0.8256944444444444in"}

Matrix ROT\_EE is multiplied by vector **k** therefore only last columns
is further applied. Finally the wrist positions is given by (in this
project dG = 0.303), px,py and pz is given by user (robot trajectory) :

![](media/image8.png){width="4.468360673665792in"
height="1.2935728346456692in"}

In order to compute theta 1, theta2 and theta3 below triangle should be
evaluated.

![](media/image9.png){width="3.4837390638670165in"
height="2.5704680664916886in"}

Analyzing the below triangle with the position of wrist WC,
calculating theta1 can be done as follows:

![](media/image10.png){width="2.231213910761155in"
height="0.47698272090988625in"}

Applying trigonometric transformations (below) the theta2 and theta3 can
be computed as follows:

![](media/image11.png){width="6.531944444444444in"
height="0.9465277777777777in"}

![](media/image12.png){width="6.531944444444444in" height="0.9375in"}

a.  **Orientation inverse kinematic problem**

Since the overall RPY (Roll Pitch Yaw) rotation between base\_link and
gripper\_link must be equal to the product of individual rotations
between respective links, following holds true:

**R0\_6 = R** (Euler Rotation matrix: roll, pitch, yaw).

**R = R0\_3 \* R3\_6**

**R3\_6 = inv(R0\_3) \* R = (R0\_3).T \* R**

**R3\_6 = (R0\_3).T \* R := U,** where U is a Euler Matrix

![](media/image13.png){width="5.815181539807524in"
height="0.8166972878390201in"}

Solving above equation (taking into account rotation matrix R3\_6 and
Euler Matrix) theta4, theta5 and theta6 are given by following
equations:

![](media/image14.png){width="6.531944444444444in"
height="1.6416666666666666in"}

**Project Implementation**

In IK\_Server.py the Python code with implemented solution was included.
Solution calculates Inverse Kinematics based on previously performed
Kinematic Analysis. Robot successfully fulfill project requirements and
complete 8/10 pick and place cycles.

![](media/image15.jpeg){width="5.091666666666667in"
height="3.928472222222222in"}


