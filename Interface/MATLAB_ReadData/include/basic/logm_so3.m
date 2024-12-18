function so3mat = logm_so3(R)
%LOGM_SO3 이 함수의 요약 설명 위치
%   자세한 설명 위치

% acosinput = (trace(R) - 1) / 2;
% if acosinput >= 1
%     so3mat = zeros(3);
% elseif acosinput <= -1
%     if ~NearZero(1 + R(3, 3))
%         omg = (1 / sqrt(2 * (1 + R(3, 3)))) ...
%               * [R(1, 3); R(2, 3); 1 + R(3, 3)];
%     elseif ~NearZero(1 + R(2, 2))
%         omg = (1 / sqrt(2 * (1 + R(2, 2)))) ...
%               * [R(1, 2); 1 + R(2, 2); R(3, 2)];
%     else
%         omg = (1 / sqrt(2 * (1 + R(1, 1)))) ...
%               * [1 + R(1, 1); R(2, 1); R(3, 1)];
%     end
%     so3mat = hat_so3(pi * omg);
% else
%    theta = acos(acosinput);
%     so3mat = theta * (1 / (2 * sin(theta))) * (R - R');
% end


so3mat = hat_so3(logm_vec(R));

end

