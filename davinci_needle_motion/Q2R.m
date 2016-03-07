function [RotMat] = Q2R(Quat)
%#codegen
%This function converts a 4x1 Rotation Quaternion (Q) to a 3x3 rotation Matrix  

a = Quat(1);
b = Quat(2);
c = Quat(3);
d = Quat(4);




RotMat = [a^2+b^2-c^2-d^2    2*b*c-2*a*d 2*b*d+2*a*c; 
          2*b*c+2*a*d        a^2-b^2+c^2-d^2 2*c*d-2*a*b;
          2*b*d-2*a*c    2*c*d+2*a*b a^2-b^2-c^2+d^2];
      


end
