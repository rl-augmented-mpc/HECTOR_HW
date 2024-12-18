function R = quat2R_casadi(q)
%QUAT2R Converts a quaternion orientation to a rotation matrix
%
import casadi.*

x=q(2);
y=q(3);
z=q(4);
w=q(1);

R = SX.zeros(3,3)    ;


    R(1,1) = 1-2*y^2-2*z^2;
    R(1,2) = 2*x*y-2*w*z;
    R(1,3) = 2*x*z+2*w*y;
    R(2,1) = 2*x*y+2*w*z;
    R(2,2) = 1-2*x^2-2*z^2;
    R(2,3) = 2*y*z-2*w*x;
    R(3,1) = 2*x*z-2*w*y;
    R(3,2) = 2*y*z+2*w*x;
    R(3,3) = 1-2*x^2-2*y^2;
end