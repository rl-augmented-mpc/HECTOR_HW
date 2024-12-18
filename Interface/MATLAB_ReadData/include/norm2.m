function out = norm2(x)
%HAT_JH Summary of this function goes here
%   Detailed explanation goes here
sum = 0;
for i=1:size(x,1)
    
    for j=1:size(x,2)
        
        
        sum = sum + x(i,j)*x(i,j);
        
        
    end
    
end

out = sqrt(sum);
end

