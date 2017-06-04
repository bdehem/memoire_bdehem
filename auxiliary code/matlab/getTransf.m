function [R, L] = getTransf(epi, u)
L = [1 0 -u(1); 0 1 -u(2); 0 0 1];
theta = atan2(epi(3)*u(2) - epi(2), epi(1)-epi(3)*u(1));
R = rollPitchYawToRotationMatrix(0,0,theta)';
end

