%% subfunction to calculate sweeping force in tissue/spatial frame
function [sweep_force_next_time, wft] = calSweepForceInTissueFrm(structNeedleGeometry,...
            K, transMatFrmNeedleToTissue, spVelocity, integral_limit, sweep_force_current_time, current_time_step, next_time_step)
    % wft in current time step
    wft = incrementalSweepForce(structNeedleGeometry, K, transMatFrmNeedleToTissue, spVelocity, integral_limit);
%     fun = @(tao) wft;
    
    delta_time = next_time_step - current_time_step;
    
    % sweepinf force in next time step
    sweep_force_next_time = sweep_force_current_time + delta_time * wft;
    
end