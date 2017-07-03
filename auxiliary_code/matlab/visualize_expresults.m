[c1,c2,proj] = get_expresults( );
makeplots = 0;
    
    
npts = size(c1,1);
pt_out = zeros(3,npts);
for i = 1:npts    

    pt1 = c1(i,:)';
    pt2 = c2(i,:)';
    
    pt_out(:,i) = triangulate(pt1, pt2);
end


if (makeplots)
    figure;
        scatter(c1(:,1),c1(:,2))
        axis ij
    figure;
        scatter(c2(:,1),c2(:,2))
        axis ij

    figure;
        scatter3(proj(:,1),proj(:,2),proj(:,3))
        axis equal
    figure;
        scatter3(pt_out(1,:),pt_out(2,:),pt_out(3,:))
        axis equal
end