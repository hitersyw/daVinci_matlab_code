function [sweep_torque_next_time, wmt] = calSweepTorqueInTissueFrm(structNeedleGeometry,...
            K, transMatFrmNeedleToTissue, spVelocity, integral_limit, sweep_torque_current_time, current_time_step, next_time_step )
% wmt in current time step
    wmt = incrementalSweepTorque(structNeedleGeometry, K, transMatFrmNeedleToTissue, spVelocity, integral_limit);
%     fun = @(tao) wft;
    
    delta_time = next_time_step - current_time_step;
    
    % sweepinf force in next time step
    sweep_torque_next_time = sweep_torque_current_time + delta_time * wmt;

end

