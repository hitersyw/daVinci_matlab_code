%% main function
function cameraIntrinsicPropertiesCal_V1()
    clear all;
    point_coordinate_wrt_world_frame = load('point_coordinate_wrt_world_frame.m');
    uv_vals_left = load('uv_vals_left.m');
    uv_vals_right = load('uv_vals_right.m');
    
    [dx_vec, dy_vec, z_val_vec] = getPointCoordinateDiff(point_coordinate_wrt_world_frame);
    

    % left pixal difference u / v
    [du_left, dv_left] = getPixalDiff(uv_vals_left);

    % right pixal difference u / v
    [du_right, dv_right] = getPixalDiff(uv_vals_right);

    [zdu_left, zdv_left] = getZduAndZdv(point_coordinate_wrt_world_frame, uv_vals_left);

    [zdu_right, zdv_right] = getZduAndZdv(point_coordinate_wrt_world_frame, uv_vals_right);
    
    % unknown intrinsic properties (left / right)
    % unknown intrinsic properties left x
    syms focal_length_left_x lcw_left_x delta_theta_left_x;
    multi_variable_left_x = [focal_length_left_x; lcw_left_x];

    % unknown intrinsic properties left y
    syms focal_length_left_y lcw_left_y delta_theta_left_y;
    multi_variable_left_y = [focal_length_left_y; lcw_left_y];

    % unknown intrinsic properties right x
    syms focal_length_right_x lcw_right_x delta_theta_right_x;
    multi_variable_right_x = [focal_length_right_x; lcw_right_x];

    % unknown intrinsic properties right y
    syms focal_length_right_y lcw_right_y delta_theta_right_y;
    multi_variable_right_y = [focal_length_right_y; lcw_right_y];

    % solve linear system of equations for left x, including four methods, and results are same
    comat_left_x = getCoefficientMat(dx_vec, du_left);
    multi_variable_left_x = linsolve(comat_left_x, zdu_left);
    focal_length_left_x = multi_variable_left_x(1);
    lcw_left_x = multi_variable_left_x(2);

    % solve linear system of equations for left y
    comat_left_y = getCoefficientMat(dy_vec, dv_left);
    multi_variable_left_y = linsolve(comat_left_y, zdu_left);
    focal_length_left_y = multi_variable_left_y(1);
    lcw_left_y = multi_variable_left_y(2);

    % solve linear system of equations for right x
    comat_right_x = getCoefficientMat(dx_vec, du_right);
    multi_variable_right_x = linsolve(comat_right_x, zdu_left);
    focal_length_right_x = multi_variable_right_x(1);
    lcw_right_x = multi_variable_right_x(2);

    % solve linear system of equations for right y
    comat_right_y = getCoefficientMat(dy_vec, dv_right);
    multi_variable_right_y = linsolve(comat_right_y, zdu_left);
    focal_length_right_y = multi_variable_right_y(1);
    lcw_right_y = multi_variable_right_y(2);

    % find delta_theta estimation left / right
    [du_vec_1_and_4_left, dv_vec_1_and_4_left] = getPixalDiff_1_and_4(uv_vals_left);

    [du_vec_1_and_4_right, dv_vec_1_and_4_right] = getPixalDiff_1_and_4(uv_vals_right);
    
    [delta_theta_vec_left, delta_theta_left] = getDeltaTheta_1(du_vec_1_and_4_left, dv_vec_1_and_4_left);
    [delta_theta_vec_right, delta_theta_right] = getDeltaTheta_1(du_vec_1_and_4_right, dv_vec_1_and_4_right);

    % based on delta_ theta estimation re-calculate focal_length and lcw for both left (x / y) / right (x / y)
    % left x
    b_left_x = cos(delta_theta_left) * zdu_left - sin(delta_theta_left) * zdv_left;
    comat_left_x = getCoefficientMat_1(dx_vec, du_left, dv_left, delta_theta_left, 'x');
    
    multi_variable_left_x = linsolve(comat_left_x, b_left_x);
    focal_length_left_x_est = multi_variable_left_x(1);
    lcw_left_x_est = multi_variable_left_x(2);

    % left y
    b_left_y = sin(delta_theta_left) * zdu_left + cos(delta_theta_left) * zdv_left;
    comat_left_y = getCoefficientMat_1(dy_vec, du_left, dv_left, delta_theta_left, 'y');
    
    multi_variable_left_y = linsolve(comat_left_y, b_left_y);
    focal_length_left_y_est = multi_variable_left_y(1);
    lcw_left_y_est = multi_variable_left_y(2);

    % right x
    b_right_x = cos(delta_theta_right) * zdu_right - sin(delta_theta_right) * zdv_right;
    comat_right_x = getCoefficientMat_1(dx_vec, du_right, dv_right, delta_theta_right, 'x');
    
    multi_variable_right_x = linsolve(comat_right_x, b_right_x);
    focal_length_right_x_est = multi_variable_right_x(1);
    lcw_right_x_est = multi_variable_right_x(2);

    % right y
    b_right_y = sin(delta_theta_right) * zdu_right + cos(delta_theta_right) * zdv_right;
    comat_right_y = getCoefficientMat_1(dy_vec, du_right, dv_right, delta_theta_right, 'y');
    
    multi_variable_right_y = linsolve(comat_right_y, b_right_y);
    focal_length_right_y_est = multi_variable_right_y(1);
    lcw_right_y_est = multi_variable_right_y(2);

    focal_length_left_x
    focal_length_left_x_est
    
    focal_length_left_y
    focal_length_left_y_est
    
    focal_length_right_x
    focal_length_right_x_est

    focal_length_right_y
    focal_length_right_y_est


     % solve delta_theta_left_x
    [cos_delta_theta_left_x, sine_delta_theta_left_x] = getDeltaTheta(focal_length_left_x_est, lcw_left_x_est,...
    dx_vec, du_left, dv_left, zdu_left, zdv_left, 'x');

     % solve delta_theta_left_y
    [cos_delta_theta_left_y, sine_delta_theta_left_y] = getDeltaTheta(focal_length_left_y_est, lcw_left_y_est,...
    dy_vec, du_left, dv_left, zdu_left, zdv_left, 'y');
     
     % solve delta_theta_right_x
    [cos_delta_theta_right_x, sine_delta_theta_right_x] = getDeltaTheta(focal_length_right_x_est, lcw_right_x_est,...
    dx_vec, du_right, dv_right, zdu_right, zdv_right, 'x');

     % solve delta_theta_right_y
    [cos_delta_theta_right_y, sine_delta_theta_right_y] = getDeltaTheta(focal_length_right_y_est, lcw_right_y_est,...
    dy_vec, du_right, dv_right, zdu_right, zdv_right, 'y');

%     % solve delta_theta_left_x
%     [delta_theta_left_x_vec, delta_theta_left_x] = getDeltaTheta(focal_length_left_x, lcw_left_x,...
%                 dx_vec, du_left, dv_left, zdu_left, zdv_left, 'x');
% 
%     % solve delta_theta_left_y
%     [delta_theta_left_y_vec, delta_theta_left_y] = getDeltaTheta(focal_length_left_y, lcw_left_y,...
%                 dy_vec, du_left, dv_left, zdu_left, zdv_left, 'y');
%     
%     % solve delta_theta_right_x
%     [delta_theta_right_x_vec, delta_theta_right_x] = getDeltaTheta(focal_length_right_x, lcw_right_x,...
%                 dx_vec, du_right, dv_right, zdu_right, zdv_right, 'x');
% 
%     % solve delta_theta_right_y
%     [delta_theta_right_y_vec, delta_theta_right_y] = getDeltaTheta(focal_length_right_y, lcw_right_y,...
%                 dy_vec, du_right, dv_right, zdu_right, zdv_right, 'y');

end
%% subfunc getting the coordinate difference of points wrt world frame
function [dx_vec, dy_vec, z_val_vec] = getPointCoordinateDiff(input_mat)
    dx_vec = [];
    dy_vec = [];
    z_val_vec = [];
    for i = 1:4:21
        dx = input_mat(i, 1) - input_mat(i + 3, 1);
        dy = input_mat(i + 1, 2) - input_mat(i, 2);
        z_val = input_mat(i, 3);
        dx_vec = [dx_vec; dx];
        dy_vec = [dy_vec; dy];
        z_val_vec = [z_val_vec; z_val];
        dx = input_mat(i + 1, 1) - input_mat(i + 2, 1);
        dy = input_mat(i + 2, 2) - input_mat(i + 3, 2);
        z_val = input_mat(i + 1, 3);
        dx_vec = [dx_vec; dx];
        dy_vec = [dy_vec; dy];
        z_val_vec = [z_val_vec; z_val];
    end
end

%% subfunc getting left/right du and dv
function [du_vec_1_and_4, dv_vec_1_and_4] = getPixalDiff_1_and_4(input_mat)
    du_vec_1_and_4 = [];
    dv_vec_1_and_4 = [];
    for i = 1:4:21
        du = input_mat(i, 1) - input_mat(i + 3, 1);
        dv = input_mat(i, 2) - input_mat(i + 3, 2);
        du_vec_1_and_4 = [du_vec_1_and_4; du];
        dv_vec_1_and_4 = [dv_vec_1_and_4; dv];
        du = input_mat(i + 1, 1) - input_mat(i + 2, 1);
        dv = input_mat(i + 1, 2) - input_mat(i + 2, 2);
        du_vec_1_and_4 = [du_vec_1_and_4; du];
        dv_vec_1_and_4 = [dv_vec_1_and_4; dv];
    end
end

%% subfunc getting delta theta estimations
function [delta_theta_vec, delta_theta] = getDeltaTheta_1(du_vec_1_and_4, dv_vec_1_and_4)
    delta_theta_vec = [];
    for i = 1:size(du_vec_1_and_4, 1)
        delta_theta = dv_vec_1_and_4(i) / du_vec_1_and_4(i);
        delta_theta_vec = [delta_theta_vec; delta_theta];
    end
    delta_theta_sum = 0;
    for i = 1:size(delta_theta_vec, 1)
        delta_theta_sum = delta_theta_sum + delta_theta_vec(i);
    end
    delta_theta = delta_theta_sum / size(delta_theta_vec, 1);
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

%% subfunc for fomatting coeficient matrix
function co_mat = getCoefficientMat_1(coordinate_diff, u_pixal_diff,...
    v_pixal_diff, delta_theta, coord_diff_direction)
    co_mat = [];
    switch coord_diff_direction
        case 'x'
            for i = 1:size(coordinate_diff, 1)
                co_mat(i,1) = coordinate_diff(i);
                co_mat(i,2) = -(cos(delta_theta) * u_pixal_diff(i) - sin(delta_theta) * v_pixal_diff(i));
            end
        case 'y'
            for i = 1:size(coordinate_diff, 1)
                co_mat(i,1) = coordinate_diff(i);
                co_mat(i,2) = -(sin(delta_theta) * u_pixal_diff(i) + cos(delta_theta) * v_pixal_diff(i));
            end
    end
end

%% subfunc calculating delta_theta
function [cos_delta_theta, sine_delta_theta] = getDeltaTheta(focal_length, lcw,...
    coordinate_diff, u_pixal_diff, v_pixal_diff, zdu_vec, zdv_vec, coord_diff_direction)
    
    switch coord_diff_direction
        case 'x'
            b = [];
            co_mat = [];
            for i = 1:size(coordinate_diff, 1)
                b_val = focal_length * coordinate_diff(i);
                b = [b; b_val];
                co_mat(i,1) = u_pixal_diff(i) * lcw + zdu_vec(i);
                co_mat(i,2) = -(v_pixal_diff(i) * lcw + zdv_vec(i));
            end
            result = linsolve(co_mat, b);
            cos_delta_theta = result(1);
            sine_delta_theta = result(2);
        case 'y'
            b = [];
            co_mat = [];
            for i = 1:size(coordinate_diff, 1)
                b_val = focal_length * coordinate_diff(i);
                b = [b; b_val];
                co_mat(i,1) = v_pixal_diff(i) * lcw + zdv_vec(i);
                co_mat(i,2) = u_pixal_diff(i) * lcw + zdu_vec(i);
            end
            result = linsolve(co_mat, b);
            cos_delta_theta = result(1);
            sine_delta_theta = result(2);
    end
end

% % subfunc calculating delta_theta
% function [delta_theta_vec, delta_theta] = getDeltaTheta(focal_length, lcw,...
%     coordinate_diff, u_pixal_diff, v_pixal_diff, zdu_vec, zdv_vec, coord_diff_direction)
%     delta_theta_vec = [];
% 
%     switch coord_diff_direction
%         case 'x'
%             for i = 1:12
%                 p_1 = focal_length * coordinate_diff(i);
%                 p_2 = (u_pixal_diff(i) * lcw);
%                 p_3 = zdu_vec(i);
%                 p_4 = (v_pixal_diff(i) * lcw);
%                 p_5 = zdv_vec(i);
%                 delta_theta = -((p_1 - p_2 - p_3) /  (p_4 + p_5));  % compute delta_theta in x (u) direction
%                 delta_theta_vec = [delta_theta_vec; delta_theta];  % appending delta_theta to delta_theta_vec (like push_back);
%             end
%         case 'y'
%             for i = 1:12
%                 p_1 = focal_length * coordinate_diff(i);
%                 p_2 = (v_pixal_diff(i) * lcw);
%                 p_3 = zdv_vec(i);
%                 p_4 = (u_pixal_diff(i) * lcw);
%                 p_5 = zdu_vec(i);
%                 delta_theta = ((p_1 - p_2 - p_3) /  (p_4 + p_5));  % compute delta_theta in y (u) direction;
%                 delta_theta_vec = [delta_theta_vec; delta_theta];  % appending delta_theta to delta_theta_vec (like push_back);
%             end
%     end
% 
%    all_one_array = ones(size(delta_theta_vec, 1), 1);
%    delta_theta = all_one_array \ delta_theta_vec;
% 
%     delta_theta_sum = 0;
%     for i = 1:size(delta_theta_vec, 1)
%         delta_theta_sum = delta_theta_sum + (delta_theta_vec(i))^2;
%     end
%     delta_theta = sqrt(delta_theta_sum / size(delta_theta_vec, 1));
% end
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