function pt_out = optimalTriangulate(pt1, pt2,pose1,pose2,rot1,rot2)
if nargin == 0
    fx = 529.1; fy = 529.1; f = [fx; fy];
    cx = 350.6; cy = 182.2; c = [cx; cy];
    K = [fx 0 cx; 0 fy cy; 0 0 1];

    pts3Dworld = makePointSet(2);
    npts = size(pts3Dworld,2);

    pose1 =[-10;0;0];
    pose2 =[-10;0;5];
    cam_pos = [0;0;0];
    drone2world = eye(3);
    cam2drone = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
    rot1 = drone2world*cam2drone; %cam2world
    rot2 = drone2world*cam2drone;
    allpts2D1 = project2D(pts3Dworld, f, c, cam2drone, cam_pos, drone2world, pose1);
    allpts2D2 = project2D(pts3Dworld, f, c, cam2drone, cam_pos, drone2world, pose2);

    pt1 = allpts2D1(:,1);
    pt2 = allpts2D2(:,1);
%     pt1 = [378.4474; 210.0474];
%     pt2 = [378.4474; 488.5211];
end
    
fx = 529.1; fy = 529.1; f = [fx; fy];
cx = 350.6; cy = 182.2; c = [cx; cy];
K = [fx 0 cx; 0 fy cy; 0 0 1];

p1 = [pt1; 1];
p2 = [pt2; 1];
epipole1 = rot1'*(pose2 - pose1); %pose2_in_sys1 (transpose, because wa go from world 2 cam)
epipole2 = rot2'*(pose1 - pose2); %pose1_in_sys2
%epipole1 = epipole1/epipole1(1);
%epipole2 = epipole2/epipole2(1);

[R1, L1] = getTransf( epipole1, p1 );
[R2, L2] = getTransf( epipole2, p2 );
T1 = R1'*L1
T2 = R2'*L2

transformed_epi1 = T1*epipole1;
transformed_epi2 = T2*epipole2;
transformed_p1 = T1*p1;
transformed_p2 = T2*p2;
f1 = transformed_epi1(3)/transformed_epi1(1);
f2 = transformed_epi2(3)/transformed_epi2(1);

F = getFundamentalMatrix( rot1, rot2, pose1, pose2, K, K);
%Jusqu ici OK

F_trans = T2'*F*T1
p2dh1 = [allpts2D1; ones(1,npts)];
p2dh2 = [allpts2D2; ones(1,npts)];
p2dh1'*F*p2dh2
(T1*p2dh1)'*F_trans*(T2*p2dh2);
a = F_trans(2,2);
b = F_trans(2,3);
c = F_trans(3,2);
d = F_trans(3,3);
%f1 = 0; f2 = 0;
coeff = getPolyCoefs(a,b,c,d,f1,f2);
coeff(abs(coeff)<10^(-9)) = 0;
t = roots(coeff);
optimal_p1_trans = [t*t*f1/((t*f1)^2 + 1); t/((t*f1)^2 + 1); 1];
optimal_p2_trans = [t*t*f2/((t*f2)^2 + 1); t/((t*f2)^2 + 1); 1];

optimal_p1 = T1'*optimal_p1_trans;
optimal_p2 = T2'*optimal_p2_trans;

p1 = [optimal_p1(1)/optimal_p1(3); optimal_p1(2)/optimal_p1(3)];
p2 = [optimal_p2(1)/optimal_p2(3); optimal_p2(2)/optimal_p2(3)];

pt_out  = triangulate(p1, p2, pose1,pose2,rot1,rot2);

 end






