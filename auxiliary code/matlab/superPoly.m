function [r] = superPoly(T,a,b,c,d ,f1, f2)
i = 0;
r = T;

for t = T
    i = i+1;
    at_b = a*t + b;
    ct_d = c*t + d;

    r(i) = t*(at_b*at_b + f2*f2*ct_d*ct_d)^2 ...
          - (a*d - b*c)*((1 + t*t*f1*f1)^2)*at_b*ct_d;
end
      
end

