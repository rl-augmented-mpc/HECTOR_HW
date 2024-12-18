function  x = forward_kinematics_mini_cheetah(q)
%FORWARD_KINEMATICS 이 함수의 요약 설명 위치
%   자세한 설명 위치

x1 = forward_kinematics_mini_cheetah_leg(q,1);
x2 = forward_kinematics_mini_cheetah_leg(q,2);
x3 = forward_kinematics_mini_cheetah_leg(q,3);
x4 = forward_kinematics_mini_cheetah_leg(q,4);

x=[x1;x2;x3;x4];

end

