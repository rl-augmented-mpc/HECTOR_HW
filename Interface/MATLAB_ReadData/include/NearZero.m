function out = NearZero(x)
%HAT_JH Summary of this function goes here
%   Detailed explanation goes here
out = norm2(x)<1e-10;
end

