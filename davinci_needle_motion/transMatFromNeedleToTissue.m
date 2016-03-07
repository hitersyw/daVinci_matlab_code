function [trans_mat] = transMatFromNeedleToTissue
    close
    clear
    clc
    needle = Needle_Motion;
    close
    needle_center_wrt_tissue_frame = (needle.head + needle.tail) / 2;
    p_mn = [0 -1 0 0;1 0 0 0;0 0 1 0]* (needle_center_wrt_tissue_frame - needle.frameOrigin);
    x_needle_frame = needle.tail - needle_center_wrt_tissue_frame;
    x_needle_frame = x_needle_frame(1:3)./norm(x_needle_frame);
    y_needle_frame = [0 1 0 ;-1 0 0 ;0 0 1 ]* x_needle_frame(1:3);
    z_needle_frame = cross(x_needle_frame, y_needle_frame);
    x_vec_m_frame = [0 -1 0];
    y_vec_m_frame = [1 0 0];
    z_vec_m_frame = cross(x_vec_m_frame, y_vec_m_frame);
    r_mn = calRotationMatrix(x_vec_m_frame,y_vec_m_frame,z_vec_m_frame,...
                            x_needle_frame, y_needle_frame, z_needle_frame);
end
