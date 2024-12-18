function J = jacobian_mini_cheetah(q)
%JACOBIAN_2D_ROBOT 이 함수의 요약 설명 위치
%   자세한 설명 위치
J1 = jacobian_mini_cheetah_leg(q,1);
J2 = jacobian_mini_cheetah_leg(q,2);
J3 = jacobian_mini_cheetah_leg(q,3);
J4 = jacobian_mini_cheetah_leg(q,4);

J=blkdiag(J1(1:3,1:3),J2(1:3,1:3),J3(1:3,1:3),J4(1:3,1:3));
end

