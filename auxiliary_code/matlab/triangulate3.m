function pt_out = triangulate3(pt1, pt2,origin1,origin2,R1,R2)
%http://www.iim.cs.tut.ac.jp/~kanatani/papers/sstriang.pdf (linear)
if nargin == 0
    pt1 = [378.4474; 210.0474];
    pt2 = [378.4474; 488.5211];
    origin1 =[-10;0;0];
    origin2 =[-10;0;5];
    R1 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2); %cam2world
    R2 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
end

    world2cam1 = R1;
    world2cam2 = R2;

    fx = 529.1;
    fy = 529.1;
    cx = 350.6;
    cy = 182.2;

    K = [fx 0 cx; 0 fy cy; 0 0 1];

    F = getFundamentalMatrix( R1, R2, origin1, origin2, K, K);
    pt1_h = [pt1;1];
    pt2_h = [pt2;1];
    
    P = diag([1,1,0]);
    
    x = pt1_h;
    y = pt2_h;
    den = (y'*F'*P*F*y) + (x'*F*P*F'*x);
    dpt1 = x'*F*y*P*F *y/den;
    dpt2 = x'*F*y*P*F'*x/den;
    
    pt_out = triangulate(pt1-dpt1(1:2), pt2-dpt2(1:2),origin1,origin2,R1,R2);

 end

