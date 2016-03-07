function comparePointsCoordinatesInCameraFrame()
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
    [uc_left, vc_left, uc_right, vc_right, p1_wrt_leftandright_camera_frame] = calculateCentralPixal_V1();
    px1_wrt_left_camera_frame = p1_wrt_leftandright_camera_frame(1);
    py1_wrt_left_camera_frame = p1_wrt_leftandright_camera_frame(2);
    px1_wrt_right_camera_frame = p1_wrt_leftandright_camera_frame(3);
    py1_wrt_right_camera_frame = p1_wrt_leftandright_camera_frame(4);

    pz_wrt_camer_frame = point_coordinate_wrt_world_frame(:,3) + lcw * ones(size(point_coordinate_wrt_world_frame,1), 1);
    px_wrt_left_camera_frame_est = calPointCoordinateWrtCameraFrame(u_left, uc_left, focal_length_x, pz_wrt_camer_frame)
    py_wrt_left_camera_frame_est = calPointCoordinateWrtCameraFrame(v_left, vc_left, focal_length_y, pz_wrt_camer_frame)
    px_wrt_right_camera_frame_est = calPointCoordinateWrtCameraFrame(u_right, uc_right, focal_length_x, pz_wrt_camer_frame)
    py_wrt_right_camera_frame_est = calPointCoordinateWrtCameraFrame(v_right, vc_right, focal_length_y, pz_wrt_camer_frame)
    
    px_wrt_left_camera_frame = formulatePointCoordinateWrtCameraFrame(px1_wrt_left_camera_frame, 'x');
    py_wrt_left_camera_frame = formulatePointCoordinateWrtCameraFrame(py1_wrt_left_camera_frame, 'y');
    px_wrt_right_camera_frame = formulatePointCoordinateWrtCameraFrame(px1_wrt_right_camera_frame, 'x');
    py_wrt_right_camera_frame = formulatePointCoordinateWrtCameraFrame(py1_wrt_right_camera_frame, 'y');

    error_px_wrt_left_camera_frame = px_wrt_left_camera_frame_est - px_wrt_left_camera_frame;
    error_py_wrt_left_camera_frame = py_wrt_left_camera_frame_est - py_wrt_left_camera_frame;
    error_px_wrt_right_camera_frame = px_wrt_right_camera_frame_est - px_wrt_right_camera_frame;
    error_py_wrt_right_camera_frame = py_wrt_right_camera_frame_est - py_wrt_right_camera_frame;
end

%% subfunc compute fiducials x/y coordinates in camera frame
function [pcd_wrt_camera_frame_est] = calPointCoordinateWrtCameraFrame(pixal_vec, central_pixal, focal_length, pz_wrt_camer_frame)
    pcd_wrt_camera_frame_est = [];
    for i = 1:size(pixal_vec, 1)
        pcd = ((pixal_vec(i) - central_pixal) * pz_wrt_camer_frame(i)) / focal_length
        pcd_wrt_camera_frame_est = [pcd_wrt_camera_frame_est; pcd];
    end
end

%% subfunc formulate points coordinates wrt camera frame(this is points coordinates as in physical world)
function [pcd_wrt_camera_frame] = formulatePointCoordinateWrtCameraFrame(p1_wrt_camera_frame, direction)
    pcd_wrt_camera_frame = [];

    switch direction
        case 'x'
            delta_x = 0.0254;
            point_i = [p1_wrt_camera_frame; p1_wrt_camera_frame;...
                             p1_wrt_camera_frame - delta_x; p1_wrt_camera_frame - delta_x];
            for i = 1:6
                pcd_wrt_camera_frame = [pcd_wrt_camera_frame; point_i];
            end
        case 'y'
            delta_y = 0.0254;
            point_i = [p1_wrt_camera_frame; p1_wrt_camera_frame + delta_y;...
                             p1_wrt_camera_frame + delta_y; p1_wrt_camera_frame];
            for i = 1:6
                pcd_wrt_camera_frame = [pcd_wrt_camera_frame; point_i];
            end
    end
end