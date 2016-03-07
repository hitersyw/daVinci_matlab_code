There are four versions matlab code to compute deVinci camera intrinsic values, focal length, offset, and delta theta.
The first version used fmincon function to compute intrinsic values, but the results seem wrong.

The second version used matrix equation to solve a linear systems of equation. And in this version, four sets of numbers obtained, they are: 
left_fx (left camera x direction focal length), left_lcw_x (left camera offset)
left_fy			left_lcw_y
right_fx			right_lcw_x
right_fy			right_lcw_x

For third version, either left and right camera uses the same focal length, which assumes there is no difference between fx and fy (the pixals are square) either in left and right camera.
Thus left_focal_length == right_focal_length

For fourth version, left and right camera has its own focal length, which means fx = fy for left camera and right camera. Thus left_focal_length =/= right_focal_length

For fifth version, left and right camers has same focal length and offset, but fx =\= fy, so that there are three variable
focal_length_x, focal_length_y, and lcw (offset)