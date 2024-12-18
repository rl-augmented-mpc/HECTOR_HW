function out = Adjoint(X)
	
	k = (size(X,1)-3);
	
	R = X(1:3,1:3);
	
	Adj = eye(3+3*k);
	Adj(1:3,1:3) = R;
	
	for i=1:k
		Adj(3*i+1:3*i+3, 1:3) = hat_so3( X(1:3, 3+i) ) * R;
		Adj(3*i+1:3*i+3, 3*i+1:3*i+3) = R;
	end
	
	out = Adj;
	
end

