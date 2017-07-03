function [ coeff ] = getPolyCoefs( a,b,c,d ,f, fp )
ab = a*b;
ac = a*c;
ad = a*d;
bc = b*c;
bd = b*d;
cd = c*d;
a2 = a*a;
b2 = b*b;
c2 = c*c;
d2 = d*d;
ad_bc = ad - bc;

f2  = f*f;
f4  = f2*f2;
fp2 = fp*fp;

coeff = ones(7,1);
coeff(1) = -ad_bc*f4*ac;
coeff(2) = (a2 + fp*c2)^2 - ad_bc*f4*(ad + bc);
coeff(3) = 4*(a2 + fp*c2)*(ab+fp2*cd) - ad_bc*(2*f2*ac+f4*bd);
coeff(4) = 2*(a2+fp*c2)*(b2+fp2*d2) +2*(ab-2*fp2*cd)^2 -ad_bc*2*f2*(ad+bc);
coeff(5) = 4*(ab+fp2*cd)*(b2+fp2*d2) - ad_bc*(ac+2*f2*bd);
coeff(6) = (b2 + fp2*d2)^2 - ad_bc*(ad+bc);
coeff(7) = -ad_bc*bd;

end

