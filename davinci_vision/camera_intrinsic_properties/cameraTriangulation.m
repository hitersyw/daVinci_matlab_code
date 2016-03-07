%% defining left camera frame to be reference frame

function cameraTriangulation()
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

    % get back base_line
    base_line = calculateBaseLine_V0();

    % retrive central pixal
    [uc_left, vc_left, uc_right, vc_right] = calculateCentralPixal_V1();

    % calculate disparity
    disparity_vec = calDisparity(u_left, u_right, uc_left, uc_right);

    % based on base_line, focal_length and disparity to calculate point_z_coordinate_wrt_left_frame
    pz_wrt_left_camera = calPointZcoordinate(base_line, focal_length, disparity_vec);

    height_diff = calHeightDiff(pz_wrt_left_camera);

    u_off_central_pixel_left = calPixlOffSetCentral(uc_left, u_left);
    px_wrt_left_camera = calPointXcoordinate(u_off_central_pixel_left, pz_wrt_left_camera, focal_length);

    v_off_central_pixel_left = calPixlOffSetCentral(vc_left, v_left);
    py_wrt_left_camera = calPointXcoordinate(v_off_central_pixel_left, pz_wrt_left_camera, focal_length);

    u_off_central_pixel_right = calPixlOffSetCentral(uc_right, u_right);
    px_wrt_right_camera = calPointXcoordinate(u_off_central_pixel_right, pz_wrt_left_camera, focal_length);

    v_off_central_pixel_right = calPixlOffSetCentral(vc_right, v_right);
    py_wrt_right_camera = calPointXcoordinate(v_off_central_pixel_right, pz_wrt_left_camera, focal_length);
    
    base_line_est_vec = px_wrt_left_camera - px_wrt_right_camera;
    
    p_xyz_coord_wrt_left_camera_frame = [px_wrt_left_camera py_wrt_left_camera pz_wrt_left_camera];
    p_xyz_coord_wrt_right_camera_frame = [px_wrt_right_camera py_wrt_right_camera pz_wrt_left_camera];
    
    [dx_vec_left, dy_vec_left] = getPointCoordinateDiff(p_xyz_coord_wrt_left_camera_frame);
    [dx_vec_right, dy_vec_right] = getPointCoordinateDiff(p_xyz_coord_wrt_right_camera_frame);
    
    
end

%% subfunc to calculate point_z_coordinate_wrt_left_frame
function point_z_coordinate_wrt_left_frame = calPointZcoordinate(base_line, focal_length, disparity_vec)
    point_z_coordinate_wrt_left_frame = [];
    for i = 1: size(disparity_vec, 1)
        point_z_coord_i = (base_line * focal_length) / disparity_vec(i);
        point_z_coordinate_wrt_left_frame = [point_z_coordinate_wrt_left_frame; point_z_coord_i];
    end
end

%% subfunc compute disparity
function disparity_mat = calDisparity(u_pixal_left, u_pixal_right, central_pixal_u_left, central_pixal_u_right)
    disparity_mat = [];
    for i = 1:size(u_pixal_left,1)
        disparity_i = (u_pixal_left(i) - central_pixal_u_left) - (u_pixal_right(i) - central_pixal_u_right);
        disparity_mat = [disparity_mat; disparity_i];
    end
end

%% subfunc to compute height difference
function height_diff_vec = calHeightDiff(point_z_coordinate_wrt_left_frame)
    height_diff_vec = [];
    point_z_coord_i_segment_avg = [];
    npts = size(point_z_coordinate_wrt_left_frame, 1);
    for i = 1: 4: npts - 3
        point_z_coord_i_segment = point_z_coordinate_wrt_left_frame(i: i + 3, :);
%         num = size(point_z_coord_i_segment, 1);
%         point_z_coord_i_segment_sum = 0;
%         for j = 1: num
%             point_z_coord_i_segment_sum = point_z_coord_i_segment_sum + point_z_coord_i_segment(j);
%         end
%         avg_i = point_z_coord_i_segment_sum / num;
        avg = mean(point_z_coord_i_segment);
        point_z_coord_i_segment_avg = [point_z_coord_i_segment_avg; avg];
    end

    npts = size(point_z_coord_i_segment_avg, 1);
    for i =1: npts - 1
            height_diff = point_z_coord_i_segment_avg(i + 1) - point_z_coord_i_segment_avg(i);
            height_diff_vec = [height_diff_vec; height_diff];
    end
end

%% subfunction to compute pixel offset btw central pixel
function pixel_offset_vec = calPixlOffSetCentral(central_pixel, pixel_vec)
    pixel_offset_vec = [];
    npts = size(pixel_vec, 1);
    for i = 1:npts
        offset = pixel_vec(i) - central_pixel;
        pixel_offset_vec = [pixel_offset_vec; offset];
    end
end

%% subfunction to compute point_x_coordinate_wrt_camera_frame
function point_x_coordinate_wrt_camera_frame = calPointXcoordinate(pixel_offset_vec,...
                                                point_z_coordinate_wrt_left_frame, focal_length)
    point_x_coordinate_wrt_camera_frame = [];
    npts = size(point_z_coordinate_wrt_left_frame, 1)
    for i = 1: npts
        point_x_coord_i = (pixel_offset_vec(i) * point_z_coordinate_wrt_left_frame(i)) / focal_length;
        point_x_coordinate_wrt_camera_frame = [point_x_coordinate_wrt_camera_frame; point_x_coord_i];
    end
end

%% subfunc getting the coordinate difference of points wrt world frame
function [dx_vec, dy_vec] = getPointCoordinateDiff(input_mat)
% function [dx_vec, dy_vec, z_val_vec] = getPointCoordinateDiff(input_mat)

    dx_vec = [];
    dy_vec = [];
%     z_val_vec = [];
    for i = 1:4:21
        dx = input_mat(i, 1) - input_mat(i + 3, 1);
        dy = input_mat(i + 1, 2) - input_mat(i, 2);
%         z_val = input_mat(i, 3);
        dx_vec = [dx_vec; dx];
        dy_vec = [dy_vec; dy];
%         z_val_vec = [z_val_vec; z_val];
        dx = input_mat(i + 1, 1) - input_mat(i + 2, 1);
        dy = input_mat(i + 2, 2) - input_mat(i + 3, 2);
%         z_val = input_mat(i + 1, 3);
        dx_vec = [dx_vec; dx];
        dy_vec = [dy_vec; dy];
%         z_val_vec = [z_val_vec; z_val];
    end
end