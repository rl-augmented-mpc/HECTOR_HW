function filtered = lpf_junny(des_freq,dt,now,bef);
%LPF_JUNNY Summary of this function goes here
%   Detailed explanation goes here

vector_size = size(now,1);


filternum = des_freq * dt;

for i=1:vector_size
    
    filtered(i) = filternum*now(i) + (1-filternum)*bef(i);
    
    
end
end

