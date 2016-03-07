function transPointCoordFromWorldFrameToPixel()
    clear all;
    point_coordinate_wrt_world_frame = load('point_coordinate_wrt_world_frame.m');
    point_coordinate_wrt_world_frame = [point_coordinate_wrt_world_frame ones(24,1)];
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

    trans_mat_from_left_camera_to_world_frame = ([eye(3) [0.0254-px1_wrt_left_camera_frame; -py1_wrt_left_camera_frame; -lcw]; [0 0 0 1]])
    trans_mat_from_right_camera_to_world_frame = ([eye(3) [0.0254-px1_wrt_right_camera_frame; -py1_wrt_right_camera_frame; -lcw]; [0 0 0 1]])


    projection_mat_from_world_frame_to_left_image = [focal_length_x 0 0; 0 -focal_length_y 0; 0 0 1] *...
                                         eye(3) * [eye(3) -trans_mat_from_left_camera_to_world_frame(1:3,4)];

    projection_mat_from_world_frame_to_right_image = [focal_length_x 0 0; 0 -focal_length_y 0; 0 0 1] *...
                                         eye(3) * [eye(3) -trans_mat_from_right_camera_to_world_frame(1:3,4)];

                                     
    projection_mat_from_world_frame_to_left_image = [focal_length_x 0 uc_left; 0 -focal_length_y vc_left; 0 0 1] * [eye(3) zeros(3,1)] *...
                                                    [eye(3) -trans_mat_from_left_camera_to_world_frame(1:3,4);0 0 0 1];
    projection_mat_from_world_frame_to_right_image = [focal_length_x 0 uc_right; 0 -focal_length_y vc_right; 0 0 1] * [eye(3) zeros(3,1)] *...
                                                    [eye(3) -trans_mat_from_right_camera_to_world_frame(1:3,4);0 0 0 1];

    uv_vals_left_est = transformFromWorldToImage(projection_mat_from_world_frame_to_left_image,...
                                    point_coordinate_wrt_world_frame);

    uv_vals_right_est = transformFromWorldToImage(projection_mat_from_world_frame_to_right_image,...
                                    point_coordinate_wrt_world_frame);
end

function uv_vals_est = transformFromWorldToImage(projection_mat, points_world_coordinates)
    uv_vals_est = []
    for i = 1: 24
        uv_vals_est_i = projection_mat * transpose(points_world_coordinates(i,:))
        uv_vals_est = [uv_vals_est; transpose(uv_vals_est_i)];
    end
end