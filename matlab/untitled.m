close all;

fx = 529.1; fy = 529.1; f = [fx; fy];
cx = 350.6; cy = 182.2; c = [cx; cy];

pts3Dworld = makePointSet(2);
npts = size(pts3Dworld,2);

drone_pos1 = [-10; 0; 0];
drone_pos2 = [-10; 0; 5]; 
cam_pos = [0;0;0];
drone2world = eye(3);
cam2drone = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
allpts2D1 = project2D(pts3Dworld, f, c, cam2drone, cam_pos, drone2world, drone_pos1);
allpts2D2 = project2D(pts3Dworld, f, c, cam2drone, cam_pos, drone2world, drone_pos2);

cam2world = drone2world*cam2drone;

pt_out = zeros(3,npts);
for i = 1:npts    

    pt1 = allpts2D1(:,i);
    pt2 = allpts2D2(:,i);
    
    pt_out(:,i) = triangulate(pt1, pt2, drone_pos1,drone_pos2,cam2world,cam2world);
end
pt_orig = pts3Dworld
pt_out
% 
% figure;
% scatter(allpts2D1(1,:),allpts2D1(2,:))
% xlabel('x')
% ylabel('y')
% axis ij
% 
% figure;
% scatter(allpts2D2(1,:),allpts2D2(2,:))
% xlabel('x')
% ylabel('y')
% axis ij


% for i = 1:npts
%     if pts3Dcam(3,i) == 0
%         allpts2D(1,i) = Inf*sign(pts3Dcam(1,i));
%         allpts2D(2,i) = Inf*sign(pts3Dcam(2,i));
%         allpts2D(isnan(allpts2D(:,i),i)) = 0;
%     else
%         allpts2D(1,i) = pts3Dcam(1,i)*f_cam(1)/pts3Dcam(3,i);
%         allpts2D(2,i) = pts3Dcam(2,i)*f_cam(2)/pts3Dcam(3,i);
%     end
% end


% figure;
% hold on;
% scatter3(pts3Dworld(1,:),pts3Dworld(2,:),pts3Dworld(3,:))
% xlabel('x')
% ylabel('y')
% zlabel('z')
% plot3([0 1],[0 0],[0 0],'r'); 
% plot3([0 0],[0 1],[0 0],'g'); 
% plot3([0 0],[0 0],[0 1],'b')
% 
% 
% figure;
% hold on;
% scatter3(pts3Dcam(1,:),pts3Dcam(2,:),pts3Dcam(3,:))
% xlabel('x')
% ylabel('y')
% zlabel('z')
% plot3([0 1],[0 0],[0 0],'r'); 
% plot3([0 0],[0 1],[0 0],'g'); 
% plot3([0 0],[0 0],[0 1],'b')

