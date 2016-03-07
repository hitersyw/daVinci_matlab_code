function [uv_left, uv_right] = printUVPixal();
    uv_vals_left = load('uv_vals_left.m');
    uv_vals_right = load('uv_vals_right.m');
    
    fiducial_1_left = rearrangeUVpixal(1, 21, uv_vals_left);
    fiducial_2_left = rearrangeUVpixal(2, 22, uv_vals_left);
    fiducial_3_left = rearrangeUVpixal(3, 23, uv_vals_left);
    fiducial_4_left = rearrangeUVpixal(4, 24, uv_vals_left);
    
end

%% subfunc
function fiducial_uv_pixal = rearrangeUVpixal(start_no, end_no, uv_pixal)
    fiducial_uv_pixal = [];
    for i = start_no : 4 : end_no
        f_i = uv_pixal(i,:);
        fiducial_uv_pixal = [fiducial_uv_pixal; f_i];
    end
end