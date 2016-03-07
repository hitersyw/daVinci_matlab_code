%% CalExpoMatrix
% This function is to calculate exponential matrix used to transform
% Input: transformation matrix from needle frame to tissue frame,
%        a point in tissue frame, the surface normal vector.
% 
% Output: Two angles are used to be the integral limits.
%% Main funtion
% The main function is FindTheta
function [ expo_mat ] = CalExpoMatrix(velocity,delta_time)
        
    rot_omega = delta_time * velocity.angV; % rotation vector
    
    trans_velocity = delta_time * velocity.transV; % translation vector
    
    if ( norm(rot_omega) <= 10e-5 )
        expo_mat = [eye(3) trans_velocity;0 0 0 1];
    else
        theta = norm(rot_omega);
        rot_omega = rot_omega / theta; % normaliztion
        trans_velocity = trans_velocity / theta; % normaliztion
        w_skew = wedge(rot_omega);
    
        rot_mat = eye(3) + w_skew * sin(theta) + (w_skew^2) * (1 - cos(theta));

        trans_vec = (eye(3) - rot_mat) * cross(rot_omega,trans_velocity) + ...
                rot_omega * rot_omega' * trans_velocity * theta;
            
        expo_mat = [rot_mat trans_vec;0 0 0 1]; %Calculate rigid body transformation matrix by combining Ri, Ti, and appending row(0 0 0 1)
    end
    

end

