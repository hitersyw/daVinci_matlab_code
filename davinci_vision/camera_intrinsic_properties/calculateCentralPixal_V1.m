function [uc_left, vc_left, uc_right, vc_right, p1_wrt_leftandright_camera_frame] = calculateCentralPixal_V1()
    clear all;
    point_coordinate_wrt_world_frame = load('point_coordinate_wrt_world_frame.m');
    uv_vals_left = load('uv_vals_left.m');
    uv_vals_right = load('uv_vals_right.m');

    u_left = uv_vals_left(:,1);
    v_left = uv_vals_left(:,2);
    
    u_right = uv_vals_right(:,1);
    v_right = uv_vals_right(:,2);
    
    
    % get focal_length and offset
    [focal_length_x, focal_length_y, lcw] = cameraIntrinsicPropertiesCal_V4();
    
    focal_length = (focal_length_x + focal_length_y) / 2
    delta_x = 0.0254;
    delta_y = 0.0254;
    L = point_coordinate_wrt_world_frame(:,3) + lcw * ones(size(point_coordinate_wrt_world_frame,1), 1);

%     b_left_u = getBVec(u_left, focal_length, L, delta_x, 'x');
%     b_left_v = getBVec(v_left, focal_length, L, delta_y, 'y');
%     b_right_u = getBVec(u_right, focal_length, L, delta_x, 'x');
%     b_right_v = getBVec(v_right, focal_length, L, delta_y, 'y');
    
%     co_mat_left = getCoMat(focal_length, L);
%     co_mat_right = getCoMat(focal_length, L);

%     x = co_mat_left \ b_left_u;
%     px1_wrt_left_camera_frame = x(1);
%     uc_left = x(2);
%     
%     x = co_mat_left \ b_left_v;
%     py1_wrt_left_camera_frame = x(1);
%     vc_left = x(2);
%     
%     x = co_mat_right \ b_right_u;
%     uc_right = x(2);
%     px1_wrt_right_camera_frame = x(1);
% 
%     
%     x = co_mat_right \ b_right_v;
%     vc_right = x(2);
%     py1_wrt_right_camera_frame = x(1);

    b_left_u = getBVec(u_left, focal_length_x, L, delta_x, 'x');
    b_left_v = getBVec(v_left, focal_length_y, L, delta_y, 'y');
    b_right_u = getBVec(u_right, focal_length_x, L, delta_x, 'x');
    b_right_v = getBVec(v_right, focal_length_y, L, delta_y, 'y');

    co_mat_left_x = getCoMat(focal_length_x, L);
    co_mat_left_y = getCoMat(focal_length_y, -L);
    co_mat_right_x = getCoMat(focal_length_x, L);
    co_mat_right_y = getCoMat(focal_length_y, -L);
    
    x = co_mat_left_x \ b_left_u;
    px1_wrt_left_camera_frame = x(1);
    uc_left = x(2);
    
    x = co_mat_left_y \ b_left_v;
    py1_wrt_left_camera_frame = x(1);
    vc_left = x(2);
    
    x = co_mat_right_x \ b_right_u;
    uc_right = x(2);
    px1_wrt_right_camera_frame = x(1);

    
    x = co_mat_right_y \ b_right_v;
    vc_right = x(2);
    py1_wrt_right_camera_frame = x(1);
    
    p1_wrt_leftandright_camera_frame = [px1_wrt_left_camera_frame; py1_wrt_left_camera_frame;...
            px1_wrt_right_camera_frame; py1_wrt_right_camera_frame];
end

%% subfunc for getting b_vec
function b_vec = getBVec(pixal_vec, focal_length, L, delta, direction)
    b_vec = [];
    switch direction
        case 'x'
            for i = 1: 4: (size(pixal_vec, 1) - 3)
                pixal_i = pixal_vec(i:i + 3 , :);
                for j = 1 : 4
                    if (j == 1) || (j == 2)
                        pixal_i(j) = pixal_i(j) * L(i);
                    else
                        pixal_i(j) = pixal_i(j) * L(i) + focal_length * delta;
                    end
                end
                b_vec = [b_vec; pixal_i];
            end
        case 'y'
            for i = 1: 4: (size(pixal_vec, 1) - 3)
                pixal_i = pixal_vec(i:i + 3 , :);
                for j = 1 : 4
                    if (j == 2) || (j == 3)
                        pixal_i(j) = -(pixal_i(j) * L(i) - focal_length * delta);
                    else
                        pixal_i(j) = -pixal_i(j) * L(i);
                    end
                end
                b_vec = [b_vec; pixal_i];
            end
    end
end

%% subfunc for getting coefficient matrix
function co_mat = getCoMat(focal_length, L)
    co_mat = [];
    f_part = focal_length * ones(size(L, 1), 1);
    co_mat = [f_part L];
end