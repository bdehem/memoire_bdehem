function [ F ] = getFundamentalMatrix( R1, R2, t1, t2, K1, K2)
if nargin == 0
    fx = 529.1;
    fy = 529.1;
    cx = 350.6;
    cy = 182.2;
    K1 = [fx 0 cx; 0 fy cy; 0 0 1];
    K2 = [fx 0 cx; 0 fy cy; 0 0 1];
    t1 = [0;0;0];
    t2 = [0;0;0.645];
    R1 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
    R2 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
end
assert(all(size(R1) == [3 3]), 'Error using getFundamentalMatrix: R1 must be of size [3 3]');
assert(all(size(R2) == [3 3]), 'Error using getFundamentalMatrix: R2 must be of size [3 3]');
assert(all(size(R1) == [3 3]), 'Error using getFundamentalMatrix: K1 must be of size [3 3]');
assert(all(size(R2) == [3 3]), 'Error using getFundamentalMatrix: K2 must be of size [3 3]');
assert(all(size(K1) == [3 3]), 'Error using getFundamentalMatrix: K1 must be of size [3 3]');
assert(all(size(K2) == [3 3]), 'Error using getFundamentalMatrix: K2 must be of size [3 3]');
assert(all(size(t1) == [3 1]), 'Error using getFundamentalMatrix: t1 must be of size [3 1]');
assert(all(size(t2) == [3 1]), 'Error using getFundamentalMatrix: t2 must be of size [3 1]');

t = t2 - t1;
t_hat = [0 -t(3) t(2) ; t(3) 0 -t(1) ; -t(2) t(1) 0 ];

E = R2'*t_hat*R1;
F = K2'\E/K1;


F(abs(F)<10^-9) = 0;
end

