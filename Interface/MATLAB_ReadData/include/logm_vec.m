function out = logm_vec(R)
%LOGM_VEC 이 함수의 요약 설명 위치
%   자세한 설명 위치


% logm_so3 by seungwoo

% tr=trace(R);
% acosinput = (trace(R) - 1) / 2;
% if acosinput >= 1
%     so3mat = zeros(3);
%     
% %     if acosinput > 1.0000001
% %         so3mat = zeros(3);
% %     end
%     
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
% 
% out = vee_so3(so3mat);



% logm_so3

% 
% 
% 
% 
% 
% 
% 
% 
% 
% 



out = zeros(3,1);
tr = trace(R);
acosinput = (tr-1.0)/2.0;

R11 = R(1,1);   R12 = R(1,2);   R13 = R(1,3);
R21 = R(2,1);   R22 = R(2,2);   R23 = R(2,3);
R31 = R(3,1);   R32 = R(3,2);   R33 = R(3,3);

if ((tr+1.0) < 1e-10)
    
    if (abs(R33+1.0)>1e-5)
        tmpvec = [R13 ; R23 ; 1.0 + R33];
        omega = pi / sqrt(2.0+2.0*R33) * tmpvec;
    elseif (abs(R22+1.0)>1e-5)
        tmpvec = [R12 ; 1.0+R22 ; R32];
        omega = pi/sqrt(2.0+2.0*R22) *tmpvec;
    else
        tmpvec = [1.0+R11 ; R21 ; R31];
        omega = pi/sqrt(2.0+2.0*R11) *tmpvec;
    end
else
    
    tr3 = tr-3.0;
    
%     if tr3>1e-10
%         
%         test=0;
%         
%     end
    
    if (tr3< -1e-7)
        theta = acos(acosinput);
        magnitude = theta / (2.0*sin(theta));
    else
        magnitude = 0.5 - tr3 * tr3/12.0;
    end
    
    tmpvec2 = [R32-R23 ; R13 - R31 ; R21 - R12];
    omega = magnitude * tmpvec2;
end

out = omega;




end
