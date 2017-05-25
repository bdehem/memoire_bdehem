 function pt_out = triangulate(pt1, pt2,origin1,origin2,R1,R2)
if nargin == 0
    pt1 = [378.4474; 210.0474];
    pt2 = [378.4474; 488.5211];
    origin1 =[-10;0;0];
    origin2 =[-10;0;5];
    R1 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2); %cam2world
    R2 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
end
  

%     world2cam1 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
%     world2cam2 = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
%    origin1 = [-10; 0; 0];
%    origin2 = [-10; 0; 5];
%    origin1 = [0; 0; 0];
%    origin2 = [0; 0; 0.645];
%     

    world2cam1 = R1;
    world2cam2 = R2;

    fx = 529.1;
    fy = 529.1;
    cx = 350.6;
    cy = 182.2;

    f = [fx; fy];
    c = [cx; cy];


    ray1_cam = [(pt1 - c)./f ; 1];
    ray2_cam = [(pt2 - c)./f ; 1];

    ray1 = world2cam1 * ray1_cam;
    ray2 = world2cam2 * ray2_cam;

    a = ray1' * ray1;
    b = ray1' * ray2;
    c = ray2' * ray2;
    d = ray1' * (origin1 - origin2);
    e = ray2' * (origin1 - origin2);
    denominator = (a*c) - (b*b);
    k1 = ((b*e) - (c*d))/denominator;
    k2 = ((a*e) - (b*d))/denominator;
    p1 = origin1 + k1*ray1;
    p2 = origin2 + k2*ray2;

    pt_out = (p1 + p2)/2;
 end

