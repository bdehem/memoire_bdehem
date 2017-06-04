function [R , T] = getCameraPositionMatrices(x,y,z,rotX,rotY,rotZ, front)
  yaw   = -rotZ;
  pitch = -rotY;
  roll  = -rotX;

  world2drone = rollPitchYawToRotationMatrix(roll, pitch, yaw);
  if (front)
    drone2cam = rollPitchYawToRotationMatrix(pi/2, 0, -pi / 2);
  else
    drone2cam = rollPitchYawToRotationMatrix(pi,   0, -pi / 2);
  end

  R = drone2cam * world2drone;

  T = [x, y, z];
end

