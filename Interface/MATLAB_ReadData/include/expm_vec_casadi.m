function out = expm_vec_casadi(vec)
%EXPM_VEC �� �Լ��� ��� ���� ��ġ
%   �ڼ��� ���� ��ġ
out = expm_so3_casadi(hat_so3(vec));
end

