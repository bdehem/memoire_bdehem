function rotmat = rollPitchYawToRotationMatrix(roll, pitch, yaw)
    Rx = rotationMatrixX(roll);
    Ry = rotationMatrixY(pitch);
    Rz = rotationMatrixZ(yaw);

    rotmat =  Rz * Ry * Rx;
    rotmat(abs(rotmat)<10^-9) = 0;
end



function Rx = rotationMatrixX(angle)
  Rx = [ 1.0, 0.0,         0.0       ;
         0.0, cos(angle), -sin(angle);
         0.0, sin(angle),  cos(angle)];
end


function Ry = rotationMatrixY(angle)
  Ry = [ cos(angle), 0.0, sin(angle);
         0.0,        1.0, 0.0       ;
        -sin(angle), 0.0, cos(angle)];
end


function Rz = rotationMatrixZ(angle)
  Rz = [ cos(angle), -sin(angle), 0.0;
         sin(angle),  cos(angle), 0.0;
         0.0,         0.0,        1.0];
end

