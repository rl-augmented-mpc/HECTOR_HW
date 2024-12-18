function  p = forward_kinematics(robot, q,i_leg)
%FORWARD_KINEMATICS 이 함수의 요약 설명 위치
%   자세한 설명 위치

%%FR , FL, RR, RL

if robot==0
	FR=1;
	FL=2;
	RR=3;
	RL=4;
	abadLinkLength = 0.062;
	hipLinkLength = 0.209;
	kneeLinkLength = 0.18;
	kneeLinkY_offset = 0.004;
	bodyLength = 0.2 * 2;
	bodyWidth = 0.049 * 2;
	kinematics_direction = 1;
elseif robot==1
	RR=1;
	RL=2;
	FR=3;
	FL=4;
	abadLinkLength = 0.1135;
	hipLinkLength = 0.3279;
	kneeLinkLength = 0.35;
	kneeLinkY_offset = 0.0;
	bodyLength = 0.349 * 2;
	bodyWidth = 0.1 * 2;
	kinematics_direction= -1;
end


dp = zeros(3,1);
pHip = zeros(3,1);
p = zeros(3,1);

q1 = q(1);
q2 = q(2);
q3 = q(3);


abadLocation = [bodyLength; bodyWidth; 0] * 0.5;

sideSigns = [-1;1;-1;1];

if ((i_leg == FL) || (i_leg ==FR))
    pHip(1) = abadLocation(1);
else
    pHip(1) = -abadLocation(1);
end

if ((i_leg == FL) || (i_leg ==RL))
    pHip(2) = abadLocation(2);
else
    pHip(2) = -abadLocation(2);
end


pHip(3) = abadLocation(3);

l1 = abadLinkLength;
l2 = hipLinkLength;
l3 = kneeLinkLength;
l4 = kneeLinkY_offset;
sideSign = sideSigns(i_leg);

s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);
c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);


c23 = c2 * c3 - s2 * s3;
s23 = s2 * c3 + c2 * s3;



dp(1) = kinematics_direction*(l3 * s23 + l2 * s2);
dp(2) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
dp(3) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;


p = pHip + dp;

end
