function vector = vectorize(matrix)
%MATRIX_TO_ROW switch form of matrix into vector 
%   ex) 3x3 -> (3/3/3) x 1
row = size(matrix,1);
column = size(matrix,2);
vector = zeros(row*column,1);
for i=1:column
vector((row*(i-1)+1):(row*(i))) = matrix(:,i);

end

