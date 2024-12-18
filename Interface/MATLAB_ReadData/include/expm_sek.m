function out = expm_sek(vec)
	
	k = (size(vec,1)-3)/3;
	
	phi = vec(1:3);
	R = expm_vec(phi);
	Jac_l = right_jacobian_so3(-phi);
	
	X_sek = eye(3+k,3+k);
	X_sek(1:3,1:3) = R;
	for i=1:k
		X_sek(1:3, 3+i) = Jac_l * vec(3*i+1:3*i+3);
	end
	
	out = X_sek;
	
end

