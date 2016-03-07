function [uc_left, vc_left, uc_right, vc_right] = calculateCentralPixal()
    clear all;
    points_coordinates_wrt_world_frame = load('point_coordinate_wrt_world_frame.m');
    uv_vals_left = load('uv_vals_left.m');
    uv_vals_right = load('uv_vals_right.m');
    
    u_left = uv_vals_left(:,1);
    v_left = uv_vals_left(:,2);
    
    u_right = uv_vals_right(:,1);
    v_right = uv_vals_right(:,2);
    
    % get focal_length and offset
    [focal_length_x, focal_length_y, lcw] = cameraIntrinsicPropertiesCal_V4();
    
    focal_length = (focal_length_x + focal_length_y) / 2;
    
    vec = [0;0; -lcw];
    % get transformation matrix: camera frame wrt world frame
    trans_mat_camera_frame_wrt_world_frame = [eye(3) vec; 0 0 0 1];
    all_one_vec = ones(size(points_coordinates_wrt_world_frame, 1), 1);
    points_coordinates_wrt_world_frame_with_one = [points_coordinates_wrt_world_frame all_one_vec];
    
    % get point coordinate in camera frame
    points_coordinates_wrt_camera_frame = [];
    for i = 1:size(points_coordinates_wrt_world_frame, 1)
        a = inv(trans_mat_camera_frame_wrt_world_frame) * transpose(points_coordinates_wrt_world_frame_with_one(i,:));
        points_coordinates_wrt_camera_frame(i,:) = a(1:3);
    end
    
    px_vec = points_coordinates_wrt_camera_frame(:,1);
    py_vec = points_coordinates_wrt_camera_frame(:,2);
    pz_vec = points_coordinates_wrt_camera_frame(:,3);
    
   % get central in u direction
    b_left_x= getBVec(focal_length, px_vec, pz_vec, u_left);
    b_left_y= getBVec(focal_length, py_vec, pz_vec, v_left);
    b_right_x= getBVec(focal_length, px_vec, pz_vec, u_right);
    b_right_y= getBVec(focal_length, py_vec, pz_vec, v_right);
    
    b_x = [b_left_x;b_right_x];
    b_y = [b_left_y;b_right_y];
    
    all_one_vec = ones(size(b_x, 1), 1);
    
    uc = linsolve(all_one_vec, b_x);
    vc = linsolve(all_one_vec, b_y);
    
    [uc_left_vec, uc_left] = getCentralPixal(focal_length, px_vec, pz_vec, u_left);
    [vc_left_vec, vc_left] = getCentralPixal(focal_length, py_vec, pz_vec, v_left);
    [uc_right_vec, uc_right]= getCentralPixal(focal_length, px_vec, pz_vec, u_right);
    [vc_right_vec, vc_right]= getCentralPixal(focal_length, py_vec, pz_vec, v_right);
    
end
%% subfunc for fomatting coefficient matrix
function b = getBVec(focal_length, p1_vec, pz_vec, pixal_vec)
    b = [];
    for i = 1:size(pixal_vec, 1)
       b(i,:) = ((p1_vec(i) * focal_length) - (pz_vec(i) * pixal_vec(i)))/(-pz_vec(i));
    end
end

% subfunc calculating delta_theta
function [pixal_central_vec, central_pixal] = getCentralPixal(focal_length, p1_vec, pz_vec, pixal_vec)
    pixal_central_vec = [];
    
    for i = 1:size(p1_vec,1)
        pixal_central_vec(i,:) = ((p1_vec(i) * focal_length) - (pz_vec(i) * pixal_vec(i)))/(-pz_vec(i));
    end
    central_pixal_sum = 0;
    for i = 1:size(pixal_central_vec, 1)
        central_pixal_sum = central_pixal_sum + (pixal_central_vec(i))^2;
    end
    central_pixal = sqrt(central_pixal_sum / size(pixal_central_vec, 1));
end