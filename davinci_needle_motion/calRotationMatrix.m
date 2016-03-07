function [ rotMat_1_wrt_0 ] = calRotationMatrix( x_0, y_0, z_0, x_1, y_1, z_1)
%CALROTATIONMATRIX Summary of this function goes here
%   Detailed explanation goes here
    rotMat_1_wrt_0 = [dot(x_1,x_0) dot(y_1,x_0) dot(z_1,x_0);...
                      dot(x_1,y_0) dot(y_1,y_0) dot(z_1,y_0);...
                      dot(x_1,z_0) dot(y_1,z_0) dot(z_1,z_0)];
end

