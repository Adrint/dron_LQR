function [ A , B ] = aa_matrices_AB( RHS , x , t , u , n , m )

del = 1.0e-6;

f0 = feval( RHS , x , t , u );

for j = 1 : n
  dx = zeros( n , 1 );
  dx(j) = del;
  A(:,j)=( feval( RHS , x + dx , t , u ) - f0 ) / del;
end

for j = 1 : m
  du = zeros( m , 1 );
  du(j) = del;
  B(:,j)=( feval( RHS , x , t , u + du ) - f0 ) / del;
end

endfunction