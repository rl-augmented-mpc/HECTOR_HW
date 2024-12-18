function out = ATE(observable_only_flag, true_trajectory_phi, est_trajectory_phi, true_trajectory_v, est_trajectory_v,true_trajectory_p, est_trajectory_p)

b = size(true_trajectory_phi, 2);

for i=1:b
	true_R = expm_vec(true_trajectory_phi(:,i));
	rpy_true_R = Rotation_to_Euler(true_R);
	rpy_true_R = [rpy_true_R(1), rpy_true_R(2), 0];
	yaw_X_true_R = EUL_ZYX_to_R_bw(rpy_true_R);
	
	est_R = expm_vec(est_trajectory_phi(:,i));
	rpy_est_R = Rotation_to_Euler(est_R);
	rpy_est_R = [rpy_est_R(1), rpy_est_R(2), 0];
	yaw_X_est_R = EUL_ZYX_to_R_bw(rpy_est_R);
	
	true_v = true_trajectory_v(:, i);
	est_v = est_trajectory_v(:, i);
	true_p = true_trajectory_p(:, i);
	est_p = est_trajectory_p(:, i);
    
    if(observable_only_flag)
        R_diff_vec = logm_vec( yaw_X_true_R * yaw_X_est_R' );	
    else
        R_diff_vec = logm_vec( true_R * est_R' );	
    end
	v_diff = true_v - true_R * est_R' * est_v;
	p_diff = true_p - true_R * est_R' * est_p;	

	ATE(1, i) = R_diff_vec'*R_diff_vec;
	ATE(2, i) = v_diff'*v_diff;
	ATE(3, i) = p_diff'*p_diff;
	
end

ATE_rot_mean = sqrt(mean(ATE(1, :),2));
ATE_rot_std = std(ATE(1, :));

ATE_v_mean = sqrt(mean(ATE(2, :),2));
ATE_v_std = std(ATE(2, :));

ATE_p_mean = sqrt(mean(ATE(3, :),2));
ATE_p_std = std(ATE(3, :));

out = [ATE_rot_mean, ATE_rot_std, ATE_v_mean, ATE_v_std, ATE_p_mean, ATE_p_std];

end
