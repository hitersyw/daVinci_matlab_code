%% FindTheta
% This function is used to calculate integral limit when computing sweeping 
% force
% Input: transformation matrix from needle frame to tissue frame,
%        a point in tissue frame, the surface normal vector.
%        structure of needle Geometry
% 
% Output: Two angles are used to be the integral limits
%% Main funtion
% The main function is FindTheta
function [integral_limit] = FindTheta( transMatFrmNeedleToTissue, point_q, normal_vec, structNeedleGeometry)
    
    % compute transformation matrix from tissue frame to needle frame
    transMatFrmTissueToNeedle = struct('rot',transMatFrmNeedleToTissue.rot',...
                    'trans',-transMatFrmNeedleToTissue.rot'* transMatFrmNeedleToTissue.trans);
                
    % transform normal_vec from tissue frame to needle frame
    normal_vec = transMatFrmTissueToNeedle.rot * normal_vec;
    point_q = transMatFrmTissueToNeedle.rot * point_q + transMatFrmTissueToNeedle.trans;
    
    % Two scenarios need to be consider
    % 1. There is no intersection point(s) between needle(extended) curve
    % and tissue surface
    % 2. There are intersection point(s) between needle(extended) curve and
    % tissue surface.
    
    % There is a special condition such that the normal vector of tissue
    % surface has no projection on the x-axis of needle frame, that means
    % the x value of normal_vec == 0
	if (normal_vec(1) == 0)
    % this condition means the tissue surface normal vector has no
    % projection on the x-axis of needle frame
        y_n = (point_q' * normal_vec) / normal_vec(2); 
        if (y_n > 0)
            syms x_n
            eqn = x_n^2 + y_n^2 == structNeedleGeometry.radius^2;
            solx = solve(eqn, x_n);
            if (~isreal(solx)) % check if x_n has imaginary part then do
            % this is when needle(extended) curve has no intersection with
            % tissue surface
                theta = NaN;
                integral_limit = integralLimit(theta);
            else
                x_n = solx;
                theta = [double(atan2(y_n, x_n(1))); double(atan2(y_n, x_n(2)))];
                integral_limit = integralLimit(theta);
            end
        elseif (y_n == 0 && normal_vec(2) == -1)
        % whole needle is inside the tissue
            integral_limit = struct('max',structNeedleGeometry.arc,'min',0);
        else
        % needle outside the tissue plane
            theta = NaN;
            integral_limit = integralLimit(theta);
        end
    else
        syms y_n;
        eqn = ((point_q' * normal_vec - normal_vec(2) * y_n)/normal_vec(1))^2 + y_n^2 ... 
            == structNeedleGeometry.radius^2;
        soly = solve(eqn, y_n);
        y_n = soly;
        if (~isreal(y_n)) % check if y_n has imaginary part then do
        % this is when needle(extended) curve has no intersection with
        % tissue surface
            theta = NaN;
            integral_limit = integralLimit(theta);
        else
            if ((y_n(1) * y_n(2)) > 0)
                if (y_n(1) < 0 && y_n(2) < 0)
                % no intersection
                    theta = NaN;
                    integral_limit = integralLimit(theta);
                elseif (y_n(1) > 0 && y_n(2) > 0)
                   x_n =  computeX_n(point_q, normal_vec, y_n);
                   theta = [double(atan2(y_n(1), x_n(1))); double(atan2(y_n(2), x_n(2)))];
                   integral_limit = integralLimit(theta);
                end
            elseif ((y_n(1) * y_n(2)) < 0)
                if (dot(normal_vec, [1;0;0]) > 0)
                   y_n = max(y_n);
                   x_n = computeX_n(point_q, normal_vec, y_n);
                   integral_limit = struct('max',structNeedleGeometry.arc,'min',double(atan2(y_n, x_n)));
                elseif (dot(normal_vec, [1;0;0]) < 0)
                   y_n = max(y_n);
                   x_n = computeX_n(point_q, normal_vec, y_n);
                   integral_limit = struct('max',double(atan2(y_n, x_n)),'min',0);
                end
            end
        end    
    end
end

%% subfunction find upperlimit and lower limit given theta vector(two element)
function [integral_limit] = integralLimit(theta)
    if (isnan(theta))
        integral_limit = struct('max',NaN,'min',NaN);
    else
        max = theta(1);
        min = theta(1);
        for i =1:size(theta)
            if (max < theta(i))
                max = theta(i);
            end
            if (min > theta(i))
                min = theta(i);
            end
        end
        integral_limit = struct('max',max,'min',min);
    end
end

%% subfunction
function [x_n] = computeX_n(point_q, normal_vec, y_n)
    n = length(y_n);
    x_n = ((point_q' * normal_vec) * ones(n,1) - (normal_vec(2) * y_n))./normal_vec(1);
end
