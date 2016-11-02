function G_GN = gripperFK(W, Q, G_0, theta)
    sum_joint = length(theta);
    G_GN = eye(4);
    flag = [0 1 1 1]';
    for i = 1:sum_joint
        exp_mat = rbtMatrixGen(W(:,i), Q(:,i), theta(i), flag(i))
        G_GN = G_GN * exp_mat
    end
    G_GN = G_GN * G_0;
end

%% subfunction for computing rigid body transformation matrices
function exp_mat = rbtMatrixGen(w, q, theta, flag)
    homo_row = [0 0 0 1];
    %every element in Cell R is a 3*3 matrix calculated by calling function: expoMatrix
    R = expoMatrix(w, theta, flag);
    %every element in Cell T is a 3*1 vector calculated by calling function: transVector
    T = transVector(R, w, q, theta, flag);
    %buliding cell array Exp a cell once time by retrieving one cell in cell array R and T,
    %and appending a row vector homoRow
    exp_mat = [R T;homo_row]; %every cell of Cell Array Exp is a matrix so that should use [] instead of use {}
end

%% subfunction of Rodrigues' formula
function rot_mat = expoMatrix(w, theta, flag)
    rot_mat = [];
    if flag == 0;
        rot_mat = eye(3);
    else
        %calling function skewMatrix to get the skew-symmetric matrix
        skew_mat = skewMatrix(w);
        %using Rodrigues' formula to obtain exponentials of skew-symmetric matrix
        rot_mat = eye(3) + skew_mat * sin(theta) + (skew_mat^2) * (1 - cos(theta));
    end
end
%% subfunction for calculating translation vector
function transaltion = transVector(rot_mat, w, q, theta, flag)
    transaltion = [];
    if flag == 0;
        transaltion = theta * w;
    else
        h = 0; %pitch of screw motion
        transaltion = (eye(3) - rot_mat) * q + h * theta * w;
    end
end

%% subfunction for calculating skew matrix
function w_skew = skewMatrix(omegaV)
    w_skew = [0 -omegaV(3) omegaV(2);omegaV(3) 0 -omegaV(1);-omegaV(2) omegaV(1) 0];
end