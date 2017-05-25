function pts_out = project2D(pts_in, f_cam, c_cam, cam_R, cam_T,drone_R,drone_T)
% pts_in: 3D points in world coordinates
% f_cam : [fx; fy]
% c_cam : [cx; cy]
% cam_R : rotationmatrix drone2cam
% cam_T : position of camera from drone origin in drone coordinates
% drone_R : rotationmatrix world2drone
% dronte_T : position of drone from world origin in world coordinates
if nargin == 1
    if pts_in == 1
        drone_T = [-10; 0; 0];
    else
        drone_T = [-10; 0; 5];
    end
    pts_in = makePointSet(2);
    f_cam = [529.1; 529.1];
    c_cam = [350.6; 182.2];
    cam_R = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
    cam_T = [0;0;0];
    drone_R = eye(3);
        
end

K = [f_cam(1) 0 c_cam(1);
     0 f_cam(2) c_cam(2);
     0 0 1];

assert(size(pts_in,1)  == 3,      'Error using project2D: pt_in must be of size [3 n]');
assert(all(size(f_cam) == [2 1]), 'Error using project2D: f_cam must be of size [2 1]');
assert(all(size(cam_R) == [3 3]), 'Error using project2D: cam_R must be of size [3 3]');
assert(all(size(f_cam) == [2 1]), 'Error using project2D: f_cam must be of size [2 1]');
assert(all(size(c_cam) == [2 1]), 'Error using project2D: c_cam must be of size [2 1]');

npts = size(pts_in,2);
drone2world = drone_R;
cam2drone   = cam_R;
drone_pos = drone_T;
pts3Dworld = pts_in;
world2cam = (drone2world*cam2drone)';

R = world2cam;
T = [eye(3) -drone_pos];
pts3Dworld_homog = [pts3Dworld;ones(1,npts)];
pts3Dcam = R*T*pts3Dworld_homog;

allpts2D_homog = K*pts3Dcam;
allpts2D = allpts2D_homog(1:2,:)./allpts2D_homog(3,:);

pts_out = allpts2D;
end

