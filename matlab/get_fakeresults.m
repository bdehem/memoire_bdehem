function [ c1, c2, pts3Dworld ] = get_fakeresults(index, pos1, pos2, rpy1, rpy2)
if nargin == 1
    pos1 = [-10; 0; 0];
    pos2 = [-10; 0; 5]; 
    rpy1 = [0, 0, 0];
    rpy2 = [0, 0, 0];
end

fx = 529.1; fy = 529.1; f = [fx; fy];
cx = 350.6; cy = 182.2; c = [cx; cy];
K = [fx 0 cx; 0 fy cy; 0 0 1];

pts3Dworld = makePointSet(index);

drone_pos1 = pos1;
drone_pos2 = pos2; 
cam_pos = [0;0;0];
drone2world1 = rollPitchYawToRotationMatrix(rpy1(1),rpy1(2),rpy1(3));
drone2world2 = rollPitchYawToRotationMatrix(rpy2(1),rpy2(2),rpy2(3));
cam2drone = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
c1 = project2D(pts3Dworld, f, c, cam2drone, cam_pos, drone2world1, drone_pos1);
c2 = project2D(pts3Dworld, f, c, cam2drone, cam_pos, drone2world2, drone_pos2);



end

