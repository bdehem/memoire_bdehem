close all;

fx = 529.1; fy = 529.1; f = [fx; fy];
cx = 350.6; cy = 182.2; c = [cx; cy];
K = [fx 0 cx; 0 fy cy; 0 0 1];

pts3Dworld = makePointSet(2);
npts = size(pts3Dworld,2);

drone_pos1 = [-10; 0; 0];
drone_pos2 = [-10; 0; 5]; 
cam_pos = [0;0;0];
world2drone = eye(3);
drone2cam = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
allpts2D1 = project2D(pts3Dworld, f, c, drone2cam, cam_pos, world2drone, drone_pos1);
allpts2D2 = project2D(pts3Dworld, f, c, drone2cam, cam_pos, world2drone, drone_pos2);
world2cam = world2drone*drone2cam;

[ c1, c2, proj ] = get_expresults( );

F = getFundamentalMatrix( world2cam, world2cam, drone_pos1, drone_pos2, K, K)
p1 = [allpts2D1(:,1) ; 1]
p2 = [allpts2D2(:,1) ; 1]

p1'*F*p2