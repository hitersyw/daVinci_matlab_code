%data is "high"(closer to camera--est 70mm) or "low"
%change to C/F for "close" (high) or "far" (low)
%images are "r" or "l" for right/left cameras
%positions 1,2,3 are: upper-right, lower right, lower left, respectively
% compare scenes 2,3 to get du or dx values
clear all;
% load calib_data;
% p_wrt_platform = world_coords_wrt_cam_frame;
    world_coords_wrt_cam_frame = load('point_coordinate_wrt_world_frame.m');
    uv_vals_left = load('uv_vals_left.m');
    uv_vals_right = load('uv_vals_right.m');
    p_wrt_platform = world_coords_wrt_cam_frame;
    
du_left = [];
dv_left=[];
zdu=[];
zdv=[];
dx_left=[];
for i=1:4:21
du =  uv_vals_left(i,1)-uv_vals_left(i+3,1);
du_left = [du_left;du];
dv = uv_vals_left(i+1,2)-uv_vals_left(i,2);
dv_left = [dv_left;dv ];
zval = p_wrt_platform(i,3)
zdu=[zdu;du*zval];
zdv=[zdv;dv*zval];

dx = p_wrt_platform(i,1)-p_wrt_platform(i+3,1);
dx_left=[dx_left;dx];
du=uv_vals_left(i+1,1)-uv_vals_left(i+2,1);
du_left = [du_left;du ];
zval = p_wrt_platform(i+1,3)
zdu=[zdu;du*zval];
dv = uv_vals_left(i+2,2)-uv_vals_left(i+3,2);
dv_left = [dv_left; dv];
zdv=[zdv;dv*zval];

dx = p_wrt_platform(i+1,1)-p_wrt_platform(i+2,1);
dx_left=[dx_left;dx];

end
du_left
dv_left
%dv_left./du_left
%dv_du=mean(dv_left./du_left)
%dv_du is about 0.05; ignore it
dx_left
target_mat = [du_left,zdu]

Lf_soln = target_mat\dx_left
f = 1/Lf_soln(2)
L = Lf_soln(1)*f

dy_left = dx_left;
target_mat_v = [dv_left,zdv]
Lf_soln_v = target_mat_v\dy_left

f_v = 1/Lf_soln_v(2)
L_v = Lf_soln_v(1)*f_v

f = 0.5*(f+f_v)
L = 0.5*(L+L_v)

%next, solve for Ox,Oy, u_c_left, v_c_left

%then, solve for v_c_right should = v_c_left; Ox_right = Ox_left -baseline; 
% solve for u_c_right and baseline

return

u_r3C = 219
u_l3C= 144

u_r2C= 555
u_l2C= 486

u_r2F = 510
u_l2F = 452

u_r3F = 237
u_l3F =176

% known distances, in mm:
dx = 25.4  %1" from position 2 to position 3
dy = 25.4  %same for dy
%dz = 15 %15mm

%L is distance from camera origin to target frame, in mm, measured along optical axis
% assume optical axis is approximately perpendicular to the target surface (i.e. normal to the sled table)
%f = L*du/dx
%f = L*(u_l2C-u_l3C)/dx
%also, f = (L+dz)*(u_l2F-u_l3F)/dx
% so, L*(u_l2C-u_l3C) = (L+dz)*(u_l2F-u_l3F)
% so, L*[(u_l2C-u_l3C) -(u_l2F-u_l3F)] = dz*(u_l2F-u_l3F)
% so, L= dz*(u_l2F-u_l3F)/[(u_l2C-u_l3C) -(u_l2F-u_l3F)]
du_l23F = u_l2F-u_l3F
du_l23C = u_l2C-u_l3C

L_on_f_left= dx/du_l23C

L_on_z_left = du_l23F/[du_l23C-du_l23F]
L_left = L_on_z_left*dz
f_left = L_left/L_on_f_left

%repeat using right-camera data:
du_r23F = u_r2F-u_r3F
du_r23C = u_r2C-u_r3C

L_on_f_right= dx/du_r23C

L_on_z_right= du_r23F/[du_r23C-du_r23F]
L_right = L_on_z_right*dz
f_right = L_right/L_on_f_right

f = (f_left+f_right)/2
L = (L_left + L_right)/2
L_on_f = L/f
L_plus_dz_on_f = (L+dz)/f


%try to find central pixels and baseline:
%b_vec = A*y_vec, where y_vec = [xl3; baseline; u_c_left; u_c_right]
%here are the dimensions of this overconstrained fit:
b_vec = zeros(8,1);
A = zeros(8,4);
y_vec = zeros(4,1);
%u_c_left,u_c_right = u-value of central pixel for left and right, resp.
%b = baseline
%xl3 = x value, in mm, w/rt left camera frame
%xr3 = xl3-b ...watch out for sign
%xl2-xl3 = dx = 2"
%vector of unknowns: xl3, baseline, u_c_left, u_c_right
%eqns: left camera
%xl3 = (L/f)*(u_l3C-u_c_left)
%       1*xl3 + 0*b +(L/f)*u_c_left + 0*u_c_right = (L/f)*u_l3C
A(1,:) = [1,0,L_on_f,0];
y_vec(1) = L_on_f*u_l3C;
%xl3 = [(L+dz)/f]*(u_l3F-u_c_left)
%       1*xl3 + 0*b + [(L+dz)/f]
% L_plus_dz_on_f*u_l3F = 1*xl3+0*b + L_plus_dz_on_f*u_c_left + 0*u_c_right
A(2,:) = [1,0,L_plus_dz_on_f,0];
y_vec(2) = L_plus_dz_on_f*u_l3F;

%xl3+dx = (L/f)*(u_l2C-u_c_left)
%L_on_f*u_l2C - dx = 1*xl3 + 0*b + L_on_f*u_c_left
A(3,:) = [1,0,L_on_f,0];
y_vec(3) = L_on_f*u_l2C-dx;
%xl3+dx = [(L+dz)/f]*(u_l2F-u_c_left)

%((L+dz)/f)u_l2F -dx = 1*xl3 + 0*b + ((L+dz)/f)*u_c_left + 0*u_c_right;
A(4,:) = [1,0,L_plus_dz_on_f,0];
y_vec(4) = L_plus_dz_on_f*u_l2F -dx;

%right camera:  x-value is offset by baseline, b
%xl3 = (L/f)*(u_r3C-u_c_right)+b ... watch sign
A(5,:) = [1,1,0,L_on_f];
y_vec(5) = L_on_f*u_r3C;
%xl3 = [(L+dz)*(u_r3F-u_c_right)+b
A(6,:) = [1,1,0,L_plus_dz_on_f];
y_vec(6) = L_plus_dz_on_f*u_r3F;
%xl3+dx = (L/f)*(u_r2C-u_c_right)+b
A(7,:) = [1,1,0,L_on_f];
y_vec(7) = L_on_f*u_r2C;
%xl3+dx = [(L+dz)/f]*(u_r2F-u_c_right) + b
A(8,:) = [1,1,0,L_plus_dz_on_f]
y_vec(8) = L_plus_dz_on_f*u_r2F

ident_vals = A\y_vec




