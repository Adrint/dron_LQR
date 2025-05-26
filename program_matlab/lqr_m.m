function [k,s]=lqr_m(a,b,q,r,nn)

%LQR	Linear quadratic regulator design for continuous-time systems.
%	[K,S] = LQR(A,B,Q,R)  calculates the optimal feedback gain matrix K
%	such that the feedback law  u = -Kx  minimizes the cost function:
%
%		J = Integral {x'Qx + u'Ru} dt
%
%	subject to the constraint equation: 
%		.
%		x = Ax + Bu 
%                
%	Also returned is S, the steady-state solution to the associated 
%	algebraic Riccati equation:
%				  -1
%		0 = SA + A'S - SBR  B'S + Q
%
%	[K,S] = LQR(A,B,Q,R,N) includes the cross-term N that relates
%	u to x in the cost function.

%	J.N. Little 4-21-85
%	Revised 8-27-86 JNL
%	Copyright (c) 1985, 1986 by the MathWorks, Inc.

%error(nargchk(4,5,nargin));
%error(abcdchk(a,b));

[m,n] = size(a);
[mb,nb] = size(b);
[mq,nq] = size(q);
if (m != mq) || (n != nq) 
	error('A and Q must be the same size')
end
[mr,nr] = size(r);
if (mr != nr) || (nb != mr)
	error('B and R must be consistent')
end

if nargin == 5
	[mn,nnn] = size(nn);
	if (mn != m) || (nnn != nr)
		error('N must be consistent with Q and R')
	end
	% Add cross term
	q = q - nn/r*nn';
	a = a - b/r*nn';
else
	nn = zeros(m,nb);
end

% Check if q is positive semi-definite and symmetric
if any(eig(q) < 0) || (norm(q'-q,1)/norm(q,1) > eps)
	error('Q must be symmetric and positive semi-definite')
end
% Check if r is positive definite and symmetric
if any(eig(r) <= 0) || (norm(r'-r,1)/norm(r,1) > eps)
	error('R must be symmetric and positive definite')
end

% Start eigenvector decomposition by finding eigenvectors of Hamiltonian:
[v,d] = eig([a b/r*b';q, -a']);
d = diag(d);
[d,index] = sort(real(d));	 % sort on real part of eigenvalues

%if (~( (d(n)<0) && (d(n+1)>0) ))
if (~( (d(n)<1.e-15) && (d(n+1)>-1.e-15) ))
   printf('Can''t order eigenvalues, (A,B) may be uncontrollable. Checking rank C = [B A*B ... A^(n-1)*B ]\n')
   C = zeros(m,n*nb);
   c = b;
   C(:,1:nb) = c;
   for i = 1 : n-1 ,
      c = a*c;
      C(:,i*nb+1:i*nb+nb) = c;
   end
   %C
   rank_C = rank(C);
   if( rank_C < n )
     rank_C
     error('rank_C < n , (A,B) are uncontrollable.')
   else
     error('rank_C = n (OK), but there is something wrong with ordering - check it out!')
   end
end

chi = v(1:n,index(1:n));	 % select vectors with negative eigenvalues
lambda = v((n+1):(2*n),index(1:n));
s = -real(lambda/chi);
k = r\(nn'+b'*s);

end