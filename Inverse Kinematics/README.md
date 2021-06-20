<h3>Inverse Kinematics</h3>
The inverse kinematics problem is the opposite of the forward kinematics problem: Given the desired 
end effector pose determine the joint configuration to achieve that pose. Inverse kinematics either 
rely on a closed-form solution or a numerical solution. Analytical solutions provide a set of equations
that fully describe the connection between the end effector position and the joint angles. For standard
serial manipulators, such as robot arms with a spherical wrist closed form solutions of the inverse 
kinematics exist. Numerical solutions are universal as they rely on numerical algorithms, and provide
solutions even if no closed-form solution is available. For a given pose there might be multiple 
solutions, e.g. elbow-up and elbow-down posture, or no solution at all, e.g. if the end effector pose 
is outside the manipulators workspace. For a numerical solution the inverse kinematics problem is 
formulated as an optimization problem. In fact the objective is to minimize the error between the 
target transform H<sub>t</sub> for the end effector and the forward kinematics of a joint 
configuration H<sub>e</sub>(q). For that purpose we decompose the pose error into a position and a 
rotation part. The position error is simply the distance of the origin of both transform:
        

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
<img src="https://render.githubusercontent.com/render/math?math=e_p(q) = [e_x, e_y, e_z]^T = [p_{xt}-p_{xe}, p_{yt}-p_{ye}, p_{zt}-p_{ze}]^T "><br/>

For the rotation part the relative orientation matrix R<sub>d</sub> = R<sub>t</sub>R<sub>e</sub>(q)' 
is converted to an axis angle representation [α w<sub>x</sub> w<sub>y</sub> w<sub>z</sub>]<sup>T</sup>.
The orientation error is given b:
<img src="https://render.githubusercontent.com/render/math?math=e_w(q) = [e_{wx}, e_{wy}, e_{wz}]^T = [\alpha w_x,\alpha w_y,\alpha w_z]^T "><br/>
The Robotics system toolbox provides a helper function to calculate the error vector e between two
transforms <b>robotics.manip.internal.IKHelpers.poseError(tformt, tformq)<\b>. The objective is to
minimize the error norm in the least squares sense:
<img src="https://render.githubusercontent.com/render/math?math=min \frac{1}{2}||e(q)|| = min \frac{1}{2}(e_x^2 %2B e_y^2 %2B e_z^2 %2B e_{xw}^2 %2B e_{wy}^2 %2B e_{wz}^2)"><br/>
A more general error is obtained by weighting the individual errors:
<img src="https://render.githubusercontent.com/render/math?math=min \frac{1}{2}e_w(q) = min \frac{1}{2} e'We"><br/>
in which W is a positive definite matrix, often diagonal. In fact if a feasible solution q* of the 
inverse kinematics problem exists, namely the target pose is within the robot workspace then the pose
 error becomes zero e(q*) = 0.<br><br/>
This problem constitutes a non-linear least squares problem for which efficient optimization
algorithms exist. The Jacobian is the matrix of first order derivatives of a vector valued function.
In our case we are interested in partial derivatives of the error vector w.r.t. to joint angles that
form the Jacobian J. The current solution q is improved with a Levenberg-Marquardt (damped least 
squares) step q' = q + ∆q with ∆q obtained from the algebraic solution of (J<sup>T</sup>J + λI)∆q =
J<sup>T</sup>e.




<h3>Rotation Matrices</h3>
Rotations are represented by 3 × 3 rotation matrices and they not only describes the relative 
orientation between frames but also performs a rotation of a vector in Euclidean space:
<p align="left">
  <img src="Figures/Rotation Matrix.JPG" width="200" title="hover text">
</p>