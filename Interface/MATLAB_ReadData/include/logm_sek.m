function out = logm_sek(X)
	
	k = (size(X,1)-3);
	
	phi = logm_vec(X(1:3,1:3));
	inv_left_jac = right_jacobian_so3(-phi)^(-1);
	
	Xi_sek = zeros(3+3*k,1);
	
	Xi_sek(1:3) = phi;
	
	for i=1:k
		Xi_sek(3*i+1:3*i+3) = inv_left_jac * X(1:3, 3+i);
	end
	
	out = Xi_sek;
	
end

