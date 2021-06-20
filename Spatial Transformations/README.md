This repository consists of some basic methods and algorithms used in Mobile Robots field taught at 
Technische Universität Dortmund. In order to implement the algorithm on Robotic applications in 
simulation, Robot Operating System(ROS) is used. The  ROS environment is launched in Ubuntu 16.04. 
In particular, we use Stage and Gazebo simulator with the configuration of the Turtlebot robot in the
Stage simulator. We also utilise Rviz as the standard ROS tool for visualization of data, for example
scan data of a range finder. The Mathworks Robotics System Toolbox connects the ROS master and 
subscribes to ROS nodes.

<h3>Spatial Transformations</h3>
Robot kinematics establishes the transformations among various coordinate frames that capture the 
positions and orientations of end-effector, links, rigid bodies and cameras as shown in figure below.
In this scenario W denotes the robot base frame or world frame, E the robot end effector frame, C the
camera frame and B the object frame. The mathematics of spatial transformations that describe rigid
motions in Euclidean space is of fundamental importance to robotic manipulation.
<p align="center">
  <img src="Figures/Coordinate Frames.JPG" width="350" title="hover text">
</p>    

<h3>Spatial Transformations</h3>
A general rigid body motion is composed of a translation and a rotation. A translation in Euclidean 
space is represented by the components of an ordinary 3D vector p = [p<sub>x</sub> p<sub>y</sub> p<sub>z</sub>]
in which the vector p = p<sub>x</sub>x + p<sub>y</sub>y + p<sub>z</sub>z. x, y, z are the unit vectors
of the frame axes of an orthogonal frame O − xyz. Successive translations are obtained by mere vector 
addition p<sub>02</sub> = p<sub>01</sub> + p<sub>12</sub>. Matlab employs the abbreviation <i>trvec</i>
for a translation vector which is represented in 3-D Euclidean space as Cartesian coordinates.

<h3>Rotation Matrices</h3>
Rotations are represented by 3 × 3 rotation matrices and they not only describes the relative 
orientation between frames but also performs a rotation of a vector in Euclidean space:
<p align="center">
  <img src="Figures/Coordinate Frames.JPG" width="250" title="hover text">
</p>    
It rotates vectors counter-clockwise through an angle θ about the z-axis of the Cartesian coordinate 
system. To perform the rotation using a rotation matrix R, the position of a point in space is 
represented by a 1-by-3 column vector p = [p<sub>x</sub> p<sub>y</sub> p<sub>z</sub>]<sup>T</sup>, that
contains the coordinates of the point w.r.t. an X − Y − Z frame. Once XYZ axes of a local frame are
expressed numerically as three direction vectors w.r.t. a global world frame, they together comprise 
the columns of the rotation matrix R that transforms vectors in the reference frame into equivalent 
vectors expressed in the coordinates of the local frame. The inverse of rotation matrix coincides with
its transpose, namely R<sup>−1</sup> = R<sup>T</sup>. Rotation matrices are square
matrices that obey the orthogonality constraints: R<sup>T</sup> = R<sup>−1</sup> and det(R) = 1.<br/><br/>
We intially define translation and rotation vectors with <i>trvec</i> function and <i>rotm</i>
respectively in different axes. Then we apply transformation p'= R<sub>x</sub>(R<sub>z</sub>(p + 
t<sub>z</sub>) + t<sub>x</sub>) 

<h3>Homogeneous Transformations</h3>
Homogeneous transformations are important as they provide the basis for defining robot direct (forward)
kinematics. The relationships between frames, e.g. robot base frame and end effector frame, are defined
by homogeneous transforms. In the field of robotics there are many possible ways of representing 
positions and orientations, but the homogeneous transformation is well matched to MATLABs powerful 
tools for matrix manipulation. Homogeneous transformations combine the two operations of rotation and 
translation into a single matrix multiplication. Homogeneous transformations are 4 × 4 matrices that
describe the relationships between Cartesian coordinate frames in terms of translation and orientation.
They are composed of a rotation matrix R and a translation vector t.
<p align="center">
  <img src="Figures/Homogenous Transformation Matrix.JPG" width="250" title="hover text">
</p>
The Robotics System Toolbox provides conversion functions for transformation representations 
tform=trvec2tform(trvec), that generates the homogeneous transformation matrix tform that corresponds
to a translation vector trvec with components t = [t<sub>x</sub>,t<sub>y</sub>,t<sub>z</sub>]. Similarly
tform=rotm2tform(rotm) generates the homogeneous transformation matrix tform (H) that corresponds to a 
rotation matrix rotm (R). We generate a homogeneous transformation tformdhc that corresponds to the
consecutive application of the translation along z-axis by d = 1, rotation along z-axis by θ = π/4, 
translation along the new x-axis by a = 1 and final rotation along the new x-axis by α = π/2. We then 
generate tformdhc by post-multiplying the transforms tformtz, tformrotz, tformtx and tformrotx i.e.
H<sub>dhc</sub> = H<sub>tz</sub>H<sub>Rz</sub>H<sub>tx</sub>H<sub>Rx</sub>

<h3>Rigid Body Tree Robot Model</h3>
Homogeneous transformations are ubiquitous in robotics and therefore in the Robotics System Toolbox.
In particular they provide the basis for defining and utilizing robot direct and inverse kinematics as
the corresponding frames are defined by homogeneous transforms. The rigid body tree model is a 
representation of a robot structure. It is useful to represent robots such as manipulators or other 
kinematic trees. The <b>RigidBodyTree</b> class provides a representation of manipulator kinematic models.
A rigid body tree is composed of rigid bodies (<b>robotics.RigidBody</b>) that are connected via joints
(<b>robotics.Joint</b>). Each rigid body has a joint that defines how that body moves relative to its parent
in the tree. The method <b>robotics.Joint.setFixedTransform</b> specifies the transformation from one 
body to the next by setting the fixed transformation on each joint. Bodies can be added, replaced,  
or removed from the rigid body tree model. You can also replace joints for specific bodies. The 
<b>RigidBodyTree</b> object maintains the relationships and updates the </b>RigidBody</b> object 
properties to reflect this relationship. The transformations between different body frames are 
retrieved with <b>robotics.RigidBodyTree.getTransform</b>. Every rigid body tree has a base. The base
defines the world coordinate frame and is the first attachment point for a rigid body. The base cannot 
be modified, except for its <b>BaseName</b> property. The rigid body is the basic building block of 
rigid body tree model and is created using <b>robotics.RigidBody</b>. Rigid body objects are added to
a rigid body tree with multiple bodies. Rigid bodies possess parent or children bodies associated with
them (Parent or Children properties). The parent is the body that this rigid body is attached to, for
example the robot base or some body upstream of the kinematic chain. The children are all the bodies 
attached to this body downstream from the base of the rigid body tree as shown in the figure below.
<p align="center">
  <img src="Figures/Rigid Body Tree.JPG" width="250" title="hover text">
</p>



The modules covered includes:
<ul>
  <li>Scan Matching</li>
  <li>Reactive Obstacle Avoidance</li>
  <li>Robot Homing</li>
  <li>Obstacle Avoidance</li>
  <li>Monte Carlo Localization</li>  
</ul>