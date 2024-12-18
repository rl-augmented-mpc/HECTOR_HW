function R = roty(theta)
%ROTX 이 함수의 요약 설명 위치
%   자세한 설명 위치

R = [cos(theta) 0 sin(theta);...
        0 1 0;...
     -sin(theta) 0 cos(theta)];
end

