%% subfunction to calculate incremental sweeping force in tissue frame
function wft = incrementalSweepForce(structNeedleGeometry, K, transMatFrmNeedleToTissue, spVelocity, integral_limit)
    syms theta;
    r = structNeedleGeometry.radius;
    s_n = [-sin(theta);cos(theta);0];
    r_n = [cos(theta);sin(theta);0];
    q_n = structNeedleGeometry.radius * r_n;
    
    s_t = transMatFrmNeedleToTissue.rot * s_n;
    q_t = transMatFrmNeedleToTissue.rot * q_n + transMatFrmNeedleToTissue.trans;
%     fun =@(theta) r * K * cross(s_t, cross((wedge(spVelocity.angV) * q_t + spVelocity.transV), s_t));
    fun = -r * K * cross(s_t, cross((wedge(spVelocity.angV) * q_t + spVelocity.transV), s_t));
    func_handle = matlabFunction(fun);
    
    wft = integral(func_handle, integral_limit.min, integral_limit.max, 'ArrayValued', true);
end