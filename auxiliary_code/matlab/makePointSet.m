function [ ptset ] = makePointSet( setnber )
if nargin == 0
    setnber = 1;
end

if (setnber==1) %smile
    pt1 = [0; -0.5; 1];
    pt2 = [0;  0.5; 1];
    pt3 = [0; -1; 0.5];
    pt4 = [0; -0.75; 0.25];
    pt5 = [0; -0.5; 0.075];
    pt6 = [0;  0  ; 0];
    pt7 = [0; 0.5 ; 0.075];
    pt8 = [0; 0.75 ; 0.25];
    pt9 = [0;  1; 0.5];
    pt10 = [0; 1 ; 1];

    ptset = [pt1 pt2 pt3 pt4 pt5 pt6 pt7 pt8 pt9 pt10];
    
elseif (setnber==2) % cube
    pt1 = [0; 0; 0];
    pt2 = [0; 0; 1];
    pt3 = [0; 1; 0];
    pt4 = [0; 1; 1];
    pt5 = [1; 0; 0];
    pt6 = [1; 0; 1];
    pt7 = [1; 1; 0];
    pt8 = [1; 1; 1];

    temp = [pt1 pt2 pt3 pt4 pt5 pt6 pt7 pt8];
    ptset = temp - repmat([0.5;0.5;0.5],1,8);
end

end

