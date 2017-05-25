function [ E ] = getEssentialMatrix( R1, R2, t1, t2)
assert(all(size(R1) == [3 3]), 'Error using getFundamentalMatrix: R1 must be of size [3 3]');
assert(all(size(R2) == [3 3]), 'Error using getFundamentalMatrix: R2 must be of size [3 3]');
assert(all(size(t1) == [3 1]), 'Error using getFundamentalMatrix: t1 must be of size [3 1]');
assert(all(size(t2) == [3 1]), 'Error using getFundamentalMatrix: t2 must be of size [3 1]');

t = t2 - t1;
t_hat = [0 -t(3) t(2) ; t(3) 0 -t(1) ; -t(2) t(1) 0 ];

E = R2'*t_hat*R1;



end

