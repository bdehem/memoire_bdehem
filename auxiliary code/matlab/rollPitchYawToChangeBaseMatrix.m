function rotmat = rollPitchYawToChangeBaseMatrix(roll, pitch, yaw)
  Rx = rotationMatrixXCB(roll);
  Ry = rotationMatrixYCB(pitch);
  Rz = rotationMatrixZCB(yaw);
  
  rotmat = Rx*Ry*Rz;
end



function Rx = rotationMatrixXCB(angle)
  Rx = [ 1.0, 0.0,         0.0       ;
         0.0, cos(angle),  sin(angle);
         0.0,-sin(angle),  cos(angle)];
end


function Ry = rotationMatrixYCB(angle)
  Ry = [ cos(angle), 0.0, -sin(angle);
         0.0,        1.0,  0.0       ;
         sin(angle), 0.0,  cos(angle)];
end


function Rz = rotationMatrixZCB(angle)
  Rz = [ cos(angle),  sin(angle), 0.0;
        -sin(angle),  cos(angle), 0.0;
         0.0,         0.0,        1.0];
end

