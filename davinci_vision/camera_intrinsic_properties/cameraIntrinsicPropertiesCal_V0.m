%% main function
function cameraIntrinsicPropertiesCal_V0()
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
    syms focus_left_x lcw_left_x delta_theta_left_x;
    multi_variable_left_x = [focus_left_x, lcw_left_x, delta_theta_left_x];

% unknown intrinsic properties left y
    syms focus_left_y lcw_left_y delta_theta_left_y;
    multi_variable_left_y = [focus_left_y, lcw_left_y, delta_theta_left_y];

% unknown intrinsic properties right x
    syms focus_right_x lcw_right_x delta_theta_right_x;
    multi_variable_right_x = [focus_right_x, lcw_right_x, delta_theta_right_x];

% unknown intrinsic properties right y
    syms focus_right_y lcw_right_y delta_theta_right_y;
    multi_variable_right_y = [focus_right_y, lcw_right_y, delta_theta_right_y];

% left x error function
    [error_func_left_x, errors_left_x] = getErrorFunc(multi_variable_left_x, dx_vec, du_left, dv_left, zdu_left, zdv_left, 'x');

% left y error function
    [error_func_left_y, errors_left_y] = getErrorFunc(multi_variable_left_y, dy_vec, du_left, dv_left, zdu_left, zdv_left, 'y');

% right x error function
    [error_func_right_x, errors_right_x] = getErrorFunc(multi_variable_right_x, dx_vec, du_right, dv_right, zdu_right, zdv_right, 'x');

% right y error function
    [error_func_right_y, errors_right_y] = getErrorFunc(multi_variable_right_y, dy_vec, du_right, dv_right, zdu_right, zdv_right, 'y');


% find camera left x intrinsic properties
    left_initial = [900 0.07 0];
    options = optimoptions('fmincon','Algorithm','active-set');
    [multi_variable_left_x, fval_left_x, exitflag_left_x,output_left_x] = fmincon...
            (@(multi_variable_left_x)getErrorFunc(multi_variable_left_x, dx_vec, du_left, dv_left, zdu_left, zdv_left, 'x'),...
            left_initial);

% find camera left y intrinsic properties
    left_initial = [900 0.07 0];
    options = optimoptions('fmincon','Algorithm','active-set');
    [multi_variable_left_y, fval_left_y, exitflag_left_y,output_left_y] = fmincon...
            (@(multi_variable_left_y)getErrorFunc(multi_variable_left_y, dy_vec, du_left, dv_left, zdu_left, zdv_left, 'y'),...
            left_initial, [], [], [], [], [], [], [], options);

% find camera right x intrinsic properties
    right_initial = [900 0.07 0];
    [multi_variable_right_x, fval_right_x, exitflag_right_x] = fmincon...
            (@(multi_variable_right_x)getErrorFunc(multi_variable_right_x, dx_vec, du_right, dv_right, zdu_right, zdv_right, 'x'),...
            right_initial);

% find camera right y intrinsic properties
    right_initial = [900 0.07 0];
    [multi_variable_right_y, fval_right_y, exitflag_right_y] = fmincon...
            (@(multi_variable_right_y)getErrorFunc(multi_variable_right_y, dy_vec, du_right, dv_right, zdu_right, zdv_right, 'y'),...
            right_initial);

% % compute errors left
%     [errors_left_sum, errors_left] = getErrorFunc(multi_variable_left, dx_vec, dy_vec, du_left, dv_left, zdu_left, zdv_left);
% 
% % comput errors right
%     [errors_right_sum, errors_right] = getErrorFunc(multi_variable_right, dx_vec, dy_vec, du_right, du_right, zdu_right, zdv_right);

% display results
    disp(multi_variable_left);
    disp(fval_left);
    disp(exitflag_left);
    
    disp(multi_variable_right);
    disp(fval_right); 
    disp(exitflag_right);

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

%% subfunc giving error for each function with unknown
function [error_func, errors_vec] = getErrorFunc(multi_variable_vec, coordinate_diff, u_pixal_diff, v_pixal_diff, zdu_vec, zdv_vec, coord_diff_direction)
    errors_vec = [];

    switch coord_diff_direction
        case 'x'
            for i = 1:12
                p_1 = multi_variable_vec(1) * coordinate_diff(i);
                p_2 = u_pixal_diff(i) * multi_variable_vec(2);
                p_3 = zdu_vec(i);
                p_4 = v_pixal_diff(i) * multi_variable_vec(3) * multi_variable_vec(2);
                p_5 = zdv_vec(i) * multi_variable_vec(3);
                error = p_1 - p_2 - p_3 + p_4 + p_5;  % unknown intrinsic function in x (u) direction;
                errors_vec = [errors_vec; error];  % appending error to errors_vec (like push_back);
            end
        case 'y'
            for i = 1:12
                p_1 = multi_variable_vec(1) * coordinate_diff(i);
                p_2 = u_pixal_diff(i) * multi_variable_vec(3) * multi_variable_vec(2);
                p_3 = zdu_vec(i) * multi_variable_vec(3);
                p_4 = v_pixal_diff(i) * multi_variable_vec(2);
                p_5 = zdv_vec(i);  % unknown intrinsic function in y (v) direction;
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