function matrix = unvectorize(vector,matrixsize)
%MATRIX_TO_ROW switch form of matrix into vector 
%   ex) 3x3 -> (3/3/3) x 1
row = matrixsize(1);
column = matrixsize(2);
matrix = zeros(row,column);
for i=1:column
matrix(:,i)=vector((row*(i-1)+1):(row*(i)));

end

