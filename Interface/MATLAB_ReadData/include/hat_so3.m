function out = hat_so3(X)
%HAT_JH Summary of this function goes here
%   Detailed explanation goes here
out = [ 0 -X(3)  X(2);
        X(3)  0 -X(1);
       -X(2)  X(1)  0 ];
end

