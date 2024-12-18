function out = RE(observable_only_flag, true_trajectory_phi, est_trajectory_phi, true_trajectory_v, est_trajectory_v,true_trajectory_p, est_trajectory_p, sub_trajectory_length)

[a, b] = size(true_trajectory_phi);

for i=1:(b-sub_trajectory_length)

	true_R_0 = expm_vec(true_trajectory_phi(:,i));
	est_R_0 = expm_vec(est_trajectory_phi(:,i));
    aligning_R = true_R_0 * est_R_0';
    
    true_v_0 = true_trajectory_v(:,i);
    est_v_0 = est_trajectory_v(:,i);
    
    true_p_0 = true_trajectory_p(:,i);
    est_p_0 = est_trajectory_p(:,i);    
    
    RE_phi_i = 0;
    RE_v_i = 0;
    RE_p_i =0;
    
    for j=1:sub_trajectory_length
        
        true_R = expm_vec(true_trajectory_phi(:,i+j));
        rpy_true_R = Rotation_to_Euler(true_R);
        rpy_true_R = [rpy_true_R(1), rpy_true_R(2), 0];
        yaw_X_true_R = EUL_ZYX_to_R_bw(rpy_true_R);
	
        est_R = expm_vec(est_trajectory_phi(:,i+j));
        rpy_est_R = Rotation_to_Euler(est_R);
        rpy_est_R = [rpy_est_R(1), rpy_est_R(2), 0];
        yaw_X_est_R = EUL_ZYX_to_R_bw(rpy_est_R);
	
        true_v = true_trajectory_v(:, i+j);
        est_v = est_trajectory_v(:, i+j);
        true_p = true_trajectory_p(:, i+j);
        est_p = est_trajectory_p(:, i+j);

        if(0)
            R_diff_vec = logm_vec( yaw_X_true_R' * aligning_R * yaw_X_est_R );	
        else
            R_diff_vec = logm_vec( true_R' * aligning_R * est_R );	
        end
        
        v_diff = est_R_0'*(est_v-est_v_0) - true_R_0'*(true_v - true_v_0);
        
        RE_phi_i = RE_phi_i + R_diff_vec'*R_diff_vec;
        RE_v_i = RE_v_i + v_diff'*v_diff;
    end
    p_diff = est_R_0'*(est_p-est_p_0) - true_R_0'*(true_p - true_p_0);
    
	RE_container(1, i) = sqrt(RE_phi_i/sub_trajectory_length);
	RE_container(2, i) = sqrt(RE_v_i/sub_trajectory_length);
	RE_container(3, i) = norm(p_diff);
	
end

RE_rot_mean = rms(RE_container(1, :));
RE_rot_std = std(RE_container(1, :));

RE_v_mean = rms(RE_container(2, :));
RE_v_std = std(RE_container(2, :));

RE_p_mean = rms(RE_container(3, :));
RE_p_std = std(RE_container(3, :));

out = [RE_rot_mean, RE_rot_std, RE_v_mean, RE_v_std, RE_p_mean, RE_p_std];

end
