close all;

fx = 529.1; fy = 529.1; f = [fx; fy];
cx = 350.6; cy = 182.2; c = [cx; cy];
K = [fx 0 cx; 0 fy cy; 0 0 1];

%drone_pos1 = [0; 0; 0];
%drone_pos2 = [0; 0; 0.645];
%drone2world = eye(3);
%cam2drone = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
%cam2world = drone2world*cam2drone;

%[allpts2D1, allpts2D2, proj] = get_expresults();
pos1 = [-10, -2, 0]';
pos2 = [-10,  2, 0]';
rpy1 = [0, 0, 0]';
rpy2 = [0, 0, 0]';
rpy1 = [0, 0, pi/6]';
rpy2 = [0, 0, -pi/6]';
drone2world1 = rollPitchYawToRotationMatrix(rpy1(1),rpy1(2),rpy1(3));
drone2world2 = rollPitchYawToRotationMatrix(rpy2(1),rpy2(2),rpy2(3));
cam2drone = rollPitchYawToRotationMatrix(-pi/2,0,-pi/2);
cam2world1 = drone2world1*cam2drone;
cam2world2 = drone2world2*cam2drone;
[allpts2D1, allpts2D2, allpts3D] = get_fakeresults(2, pos1, pos2, rpy1, rpy2);
dir1 = cam2world1*[0; 0; 1];
dir2 = cam2world2*[0; 0; 1];

P1 = K*cam2world1'*[eye(3), -pos1];
P2 = K*cam2world2'*[eye(3), -pos2];
F = getFundamentalMatrix2(P1,P2);

allpts3D_h  = [allpts3D ; ones(1,size(allpts3D,2))];
allpts2D1_h = P1*allpts3D_h;
allpts2D2_h = P2*allpts3D_h;

allpts2D1_h = allpts2D1_h./allpts2D1_h(3,:);
allpts2D2_h = allpts2D2_h./allpts2D2_h(3,:);
diag(allpts2D2_h'*F*allpts2D1_h)';


figure;
scatter3(allpts3D(1,:),allpts3D(2,:),allpts3D(3,:)); hold on;
quiver3(pos1(1),pos1(2),pos1(3),dir1(1),dir1(2),dir1(3));
quiver3(pos2(1),pos2(2),pos2(3),dir2(1),dir2(2),dir2(3));
xlabel('x')
ylabel('y')
zlabel('z')
axis equal;

figure;
scatter(allpts2D1(1,:),allpts2D1(2,:));
npts = size(allpts2D1,2);

pt_out4 = zeros(3,npts);
for i = 1:npts    
    pt1 = allpts2D1(:,i);
    pt2 = allpts2D2(:,i);
    
    pt_out4(:,i) = triangulate4(pt1, pt2, pos1, pos2, cam2world1, cam2world2);
end
allpts3D
pt_out4

figure;
hold on;
scatter3(pt_out4(1,:),pt_out4(2,:),pt_out4(3,:)); hold on;
%scatter3(pt_out1(1,:),pt_out1(2,:),pt_out1(3,:),'r')
xlabel('x')
ylabel('y')
zlabel('z')
plot3([0 1],[0 0],[0 0],'r'); 
plot3([0 0],[0 1],[0 0],'g'); 
plot3([0 0],[0 0],[0 1],'b')


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

