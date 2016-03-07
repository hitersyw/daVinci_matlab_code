function [NeedlePts] = Needle_Shape(structNeedleData,structNeedleGeometry,N,flag)

%This function generates the list of the needle points using the two
%structures above.
%structNeedleData contains 2 elements
%  1. rot, a Quaternion
%  2. trans, a positional vector.

%structNeedleGeometry contains 4 elements
%  1. straightL,  Straight Length
%  2. radius
%  3. kinkAngle 
%  4. Arc


%The Needle Points are generated using the following rule.
%1. check if the needle is straght or not.
%2. 
 
%The fourth input argument is a flag which indicates if the first input
%arguments is in quaternion format or in homogeneous matrix format
%false / 0: quaternion format
%true / 1: homogeneous matrix format

%% Generate the Needle Transformation
if (nargin == 3 || (nargin == 4 && flag == false))
    % this is when the first input is quaternion or when flag sets to false
    R_SN = Q2R(structNeedleData.rot);
    
elseif (nargin == 4 && flag == true)
    % this is when the first input is rotation matrix and flag sets to true
    R_SN = structNeedleData.rot;
else
    error('The inputs are incorrect');
end
P_SNo = structNeedleData.trans;
P_SN = zeros(3,1);
[s1 s2] = size(P_SNo);
if(s2 == 3)
    P_SN = P_SNo';
else
    P_SN = P_SNo;
end
g_SN = [R_SN P_SN; 0 0 0 1];
%% Use the needle geometric information to infer its shape.
NL = structNeedleGeometry.straightL;

if structNeedleGeometry.arc == 0
    %straight needle case

    theta = linspace(0,NL,N+1);
    %NeedlePtN1 = [theta; zeros(2,N+1);ones(1,N+1)];
    %Marks code requires z.
    NeedlePtN1 = [zeros(2,N+1);theta;ones(1,N+1);];
    
else
    %bent needle case    
    
    %Use the quaternion to develope a transform for the needle.
    %The Needle naturally lies in the x-y plane and is tangent to the -x
    %direction at first. The center vect in in the +y direction (before kink
    %angle)

    
    %NeedleL = zeros(2,N+1); 
    
    
    Rad = structNeedleGeometry.radius;
    kink = structNeedleGeometry.kinkAngle;
    
    if(NL > 0)
        theta = linspace(0,structNeedleGeometry.arc,N);
        
        NeedlePtN = [Rad*sin(kink); -Rad*(1-cos(kink)); 0; 0]*ones(1,N)+[ -NL*ones(1,N); zeros(3,N)]+[-Rad*sin(theta+kink); Rad*(1-cos(theta+kink)); zeros(1,N); ones(1,N)];
        
        NeedlePtN1 = [[0 0 0 1]' NeedlePtN];
    elseif(NL == 0 && kink == 0)
        theta = linspace(0,structNeedleGeometry.arc,N+1);
        NeedlePtN1 = [Rad*cos(theta); Rad*sin(theta); zeros(1,N+1); ones(1,N+1)];
    else
        theta = linspace(0,structNeedleGeometry.arc,N+1);
        NeedlePtN1 = [Rad*sin(kink); -Rad*(1-cos(kink)); 0; 0]*ones(1,N+1)+[ -NL*ones(1,N+1); zeros(3,N+1)]+[-Rad*sin(theta+kink); Rad*(1-cos(theta+kink)); zeros(1,N+1); ones(1,N+1)];
    end
    
    
end
    
NeedlePts = g_SN*NeedlePtN1;

end
