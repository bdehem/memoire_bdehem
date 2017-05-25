function [ F ] = getFundamentalMatrix2( P1, P2 )
X{1} = [P1(2,:); P1(3,:)];
X{2} = [P1(3,:); P1(1,:)];
X{3} = [P1(1,:); P1(2,:)];

Y{1} = [P2(2,:); P2(3,:)];
Y{2} = [P2(3,:); P2(1,:)];
Y{3} = [P2(1,:); P2(2,:)];
F = zeros(3,3);
for  i = 1:3
    for j = 1:3
        XY = [X{j}; Y{i}];
        F(i,j) = det (XY);
    end
end
end

