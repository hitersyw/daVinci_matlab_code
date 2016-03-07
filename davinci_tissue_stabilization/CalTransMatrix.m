%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    This function is used to calculate transformation matrix Gst in next %
%    time step.                                                           %
%    Input: current_trans_matrix, current_time_step, next_time_step,      %
%           hybrid_velocity.                                              %
%                                                                         %
%    Output: next_trans_matrix                                            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% main function
function [ current_trans_matrix, spVelocity ] = CalTransMatrix(previous_trans_matrix, previous_time_step, current_time_step, hyVelocity)
    needle_origin = previous_trans_matrix.trans; % the origin of needle frame wrt to tisse frame
    
    % spatial velocity is transformed from hybrid velocity
    spVelocity = calSpatialVelocity(hyVelocity, needle_origin);
    
    delta_time = current_time_step - previous_time_step; % time increment
    
    expo_mat = CalExpoMatrix(spVelocity,delta_time); % exponential matrix
    
    % next time step tranformation matrix
    trans_matrix = expo_mat * [previous_trans_matrix.rot previous_trans_matrix.trans;0 0 0 1];
    
    current_trans_matrix.rot = trans_matrix(1:3,1:3);
    current_trans_matrix.trans = trans_matrix(1:3,4);

end

%% subfunction to calculate spatial velocity given hybrid velocity
function [ spVelocity ] = calSpatialVelocity(hyVelocity, needle_origin)
    spVelocity.transV = eye(3) * hyVelocity.transV + wedge(needle_origin) * hyVelocity.angV;
    spVelocity.angV = eye(3) * hyVelocity.angV;
end