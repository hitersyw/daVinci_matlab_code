function [base_line] = calculateBaseLine_V0()
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
    focal_length = (focal_length_x + focal_length_y) / 2;
    L = point_coordinate_wrt_world_frame(:,3) + lcw * ones(size(point_coordinate_wrt_world_frame,1), 1);

    % retrive central pixal
    [uc_left, vc_left, uc_right, vc_right] = calculateCentralPixal_V1();

    pz_wrt_camer_frame = point_coordinate_wrt_world_frame(:,3) + lcw * ones(size(point_coordinate_wrt_world_frame,1), 1);
    px_wrt_left_camera_frame_est = calPointCoordinateWrtCameraFrame(u_left, uc_left, focal_length_x, pz_wrt_camer_frame)
    py_wrt_left_camera_frame_est = calPointCoordinateWrtCameraFrame(v_left, vc_left, focal_length_y, pz_wrt_camer_frame)
    px_wrt_right_camera_frame_est = calPointCoordinateWrtCameraFrame(u_right, uc_right, focal_length_x, pz_wrt_camer_frame)
    py_wrt_right_camera_frame_est = calPointCoordinateWrtCameraFrame(v_right, vc_right, focal_length_y, pz_wrt_camer_frame)

    disparity_vec = calDisparity(u_left, u_right, uc_left, uc_right);

    base_line = calBaseLine(disparity_vec, L, focal_length);
    
    error_vec = (px_wrt_left_camera_frame_est - base_line * ones(24,1)) - px_wrt_right_camera_frame_est;
end

%% subfunc compute fiducials x/y coordinates in camera frame
function [pcd_wrt_camera_frame_est] = calPointCoordinateWrtCameraFrame(pixal_vec, central_pixal, focal_length, pz_wrt_camer_frame)
    pcd_wrt_camera_frame_est = [];
    for i = 1:size(pixal_vec, 1)
        pcd = ((pixal_vec(i) - central_pixal) * pz_wrt_camer_frame(i)) / focal_length
        pcd_wrt_camera_frame_est = [pcd_wrt_camera_frame_est; pcd];
    end
end

%% subfunc compute disparity
function disparity_vec = calDisparity(u_pixal_left, u_pixal_right, central_pixal_u_left, central_pixal_u_right)
    disparity_vec = [];
    for i = 1:size(u_pixal_left,1)
        disparity_i = (u_pixal_left(i) - central_pixal_u_left) - (u_pixal_right(i) - central_pixal_u_right);
        disparity_vec = [disparity_vec; disparity_i];
    end
end

%% subfunc compute base base_line
function base_line = calBaseLine(disparity_mat, L, focal_length)
    b_vec = (L .* disparity_mat) / focal_length;

    base_line = ones(24, 1) \ b_vec;
%     base_line_sum = 0;
%     for i = 1: size(b_vec,1)
%         base_line_sum = base_line_sum + (b_vec(i))^2;
%     end
%     base_line = sqrt(base_line_sum / size(b_vec,1));

end