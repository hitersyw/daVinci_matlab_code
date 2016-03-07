%% main function
function cameraIntrinsicPropertiesCal_V3()
    clear all;
    point_coordinate_wrt_world_frame = load('point_coordinate_wrt_world_frame.m');
    uv_vals_left = load('uv_vals_left.m');
    uv_vals_right = load('uv_vals_right.m');
    
    [dx_vec, dy_vec] = getPointCoordinateDiff(point_coordinate_wrt_world_frame);
    

    % left pixal difference u / v
    [du_left, dv_left] = getPixalDiff(uv_vals_left);

    % right pixal difference u / v
    [du_right, dv_right] = getPixalDiff(uv_vals_right);

    [zdu_left, zdv_left] = getZduAndZdv(point_coordinate_wrt_world_frame, uv_vals_left);

    [zdu_right, zdv_right] = getZduAndZdv(point_coordinate_wrt_world_frame, uv_vals_right);
    
    % unknown intrinsic properties (left / right)
    % unknown intrinsic properties left x
    syms left_focal_length left_lcw left_delta_theta;
    multi_variable_left = [left_focal_length; left_lcw];

    syms right_focal_length right_lcw right_delta_theta;
    multi_variable_right = [right_focal_length; right_lcw];

    % solve linear system of equations for left x, including four methods, and results are same
    comat_left_x = getCoefficientMat(dx_vec, du_left);
    comat_left_y = getCoefficientMat(dy_vec, dv_left);
    comat_right_x = getCoefficientMat(dx_vec, du_right);
    comat_right_y = getCoefficientMat(dy_vec, dv_right);

    % combine these coefficient matrix together
    comat_left = [comat_left_x; comat_left_y];
    comat_right = [comat_right_x; comat_right_y];
    zdudv_left = [zdu_left; zdv_left];
    zdudv_right = [zdu_right; zdv_right];

    % solve left camera intrinsic values
    multi_variable_left = linsolve(comat_left, zdudv_left);
    left_focal_length = multi_variable_left(1);
    left_lcw = multi_variable_left(2);

    % solve right camera intrinsic values
    multi_variable_right = linsolve(comat_right, zdudv_right);
    right_focal_length = multi_variable_right(1);
    right_lcw = multi_variable_right(2);

    % solve delta_theta_left_x
    [delta_theta_left_x_vec, delta_theta_left_x] = getDeltaTheta(left_focal_length, left_lcw,...
                dx_vec, du_left, dv_left, zdu_left, zdv_left, 'x');

    % solve delta_theta_left_y
    [delta_theta_left_y_vec, delta_theta_left_y] = getDeltaTheta(left_focal_length, left_lcw,...
                dy_vec, du_left, dv_left, zdu_left, zdv_left, 'y');
    
    % solve delta_theta_right_x
    [delta_theta_right_x_vec, delta_theta_right_x] = getDeltaTheta(right_focal_length, right_lcw,...
                dx_vec, du_right, dv_right, zdu_right, zdv_right, 'x');

    % solve delta_theta_right_y
    [delta_theta_right_y_vec, delta_theta_right_y] = getDeltaTheta(right_focal_length, right_lcw,...
                dy_vec, du_right, dv_right, zdu_right, zdv_right, 'y');

end
%% subfunc getting the coordinate difference of points wrt world frame
function [dx_vec, dy_vec] = getPointCoordinateDiff(input_mat)
    dx_vec = [];
    dy_vec = [];
    for i = 1:4:21
        dx = input_mat(i, 1) - input_mat(i + 3, 1);
        dy = input_mat(i + 1, 2) - input_mat(i, 2);
        dx_vec = [dx_vec; dx];
        dy_vec = [dy_vec; dy];
        dx = input_mat(i + 1, 1) - input_mat(i + 2, 1);
        dy = input_mat(i + 2, 2) - input_mat(i + 3, 2);
        dx_vec = [dx_vec; dx];
        dy_vec = [dy_vec; dy];
    end
end

%% subfunc getting left/right du and dv
function [du_vec, dv_vec] = getPixalDiff(input_mat)
    du_vec = [];
    dv_vec = [];
    for i = 1:4:21
        du = input_mat(i, 1) - input_mat(i + 3, 1);
        dv = input_mat(i + 1, 2) - input_mat(i, 2);
        du_vec = [du_vec; du];
        dv_vec = [dv_vec; dv];
        du = input_mat(i + 1, 1) - input_mat(i + 2, 1);
        dv = input_mat(i + 2, 2) - input_mat(i + 3, 2);
        du_vec = [du_vec; du];
        dv_vec = [dv_vec; dv];
    end
end

%% subfunc getting left/right zdu and zdv
function [zdu_vec, zdv_vec] = getZduAndZdv(input_mat_1, input_mat_2)
    zdu_vec = [];
    zdv_vec = [];
    for i = 1:4:21
        z_val = input_mat_1(i, 3);
        du = input_mat_2(i, 1) - input_mat_2(i + 3, 1);
        dv = input_mat_2(i + 2, 2) - input_mat_2(i, 2);
        zdu_vec = [zdu_vec; du * z_val];
        zdv_vec = [zdv_vec; dv * z_val];
        z_val = input_mat_1(i + 1, 3);
        du = input_mat_2(i + 1, 1) - input_mat_2(i + 2, 1);
        dv = input_mat_2(i + 2, 2) - input_mat_2(i + 3, 2);
        zdu_vec = [zdu_vec; du * z_val];
        zdv_vec = [zdv_vec; dv * z_val];
    end
end

%% subfunc for fomatting coefficient matrix
function co_mat = getCoefficientMat(coordinate_diff, pixal_diff)
    co_mat = [];
    for i = 1:size(pixal_diff, 1)
        co_mat(i,1) = coordinate_diff(i);
        co_mat(i,2) = -pixal_diff(i);
    end
end


%% subfunc calculating delta_theta
function [delta_theta_vec, delta_theta] = getDeltaTheta(focus, lcw,...
            coordinate_diff, u_pixal_diff, v_pixal_diff, zdu_vec, zdv_vec, coord_diff_direction)
    delta_theta_vec = [];

    switch coord_diff_direction
        case 'x'
            for i = 1:12
                p_1 = focus * coordinate_diff(i);
                p_2 = (u_pixal_diff(i) * lcw);
                p_3 = zdu_vec(i) / focus;
                p_4 = (v_pixal_diff(i) * lcw);
                p_5 = zdv_vec(i) / focus;
                delta_theta = -((p_1 - p_2 - p_3) /  (p_4 + p_5));  % compute delta_theta in x (u) direction
                delta_theta_vec = [delta_theta_vec; delta_theta];  % appending delta_theta to delta_theta_vec (like push_back);
            end
        case 'y'
            for i = 1:12
                p_1 = focus * coordinate_diff(i);
                p_2 = (v_pixal_diff(i) * lcw);
                p_3 = zdv_vec(i) / focus;
                p_4 = (u_pixal_diff(i) * lcw);
                p_5 = zdu_vec(i) / focus;
                delta_theta = ((p_1 - p_2 - p_3) /  (p_4 + p_5));  % compute delta_theta in y (u) direction;
                delta_theta_vec = [delta_theta_vec; delta_theta];  % appending delta_theta to delta_theta_vec (like push_back);
            end
    end

    all_one_array = ones(size(delta_theta_vec, 1), 1);
    delta_theta = all_one_array \ delta_theta_vec;

%    delta_theta_sum = 0;
%    for i = 1:size(delta_theta_vec, 1)
%        delta_theta_sum = delta_theta_sum + (delta_theta_vec(i))^2;
%    end
%    delta_theta = sqrt(delta_theta_sum / size(delta_theta_vec, 1));
end

%% subfunc giving error for each function with unknown
function [error_func, errors_vec] = getErrorFunc(multi_variable_vec, coordinate_diff, u_pixal_diff, v_pixal_diff, zdu_vec, zdv_vec, coord_diff_direction)
    errors_vec = [];

    switch coord_diff_direction
        case 'x'
            for i = 1:12
                p_1 = multi_variable_vec(1) * coordinate_diff(i);
                p_2 = (u_pixal_diff(i) * multi_variable_vec(2)) / multi_variable_vec(1);
                p_3 = zdu_vec(i) / multi_variable_vec(1);
                p_4 = (v_pixal_diff(i) * multi_variable_vec(3) * multi_variable_vec(2)) / multi_variable_vec(1);
                p_5 = (zdv_vec(i) * multi_variable_vec(3)) / multi_variable_vec(1);
                error = p_1 - p_2 - p_3 + p_4 + p_5;  % unknown intrinsic function in x (u) direction;
                errors_vec = [errors_vec; error];  % appending error to errors_vec (like push_back);
            end
        case 'y'
            for i = 1:12
                p_1 = multi_variable_vec(1) * coordinate_diff(i);
                p_2 = (u_pixal_diff(i) * multi_variable_vec(3) * multi_variable_vec(2)) / multi_variable_vec(1);
                p_3 = (zdu_vec(i) * multi_variable_vec(3)) / multi_variable_vec(1);
                p_4 = (v_pixal_diff(i) * multi_variable_vec(2)) / multi_variable_vec(1);
                p_5 = zdv_vec(i) / multi_variable_vec(1);  % unknown intrinsic function in y (v) direction;
                error = p_1 - p_2 - p_3 - p_4 - p_5;
                errors_vec = [errors_vec; error];
            end
    end

    error_square_sum = 0;
    for i = 1:size(errors_vec, 1)
        error_square_sum = error_square_sum + (errors_vec(i))^2;
    end
    error_func = 1 / sqrt(error_square_sum);
end