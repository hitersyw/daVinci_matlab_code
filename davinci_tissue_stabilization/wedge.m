function W = wedge(w)
%wedge.m 
%
%--------------------------------------------------------------------------
%Implements the \hat map, that is, 
%creates a skew-symmetric matrix from the vector w. 
%If w is a R^3 vector, then wedge(w) is a
%3x3 matrix. w can be a row or a column vector.
%
%... Last Modified: May 27, 2010
%... Authors:       Nawaf Bou-Rabee (nawaf@cims.nyu.edu)
%                   Giulia Ortolan (ortolang@dei.unipd.it)
%                   Alessandro Saccon (asaccon@isr.ist.utl.pt)
%--------------------------------------------------------------------------
%
%==========================================================================
%w = a \Real^3 vector;
%==========================================================================

W = [0 -w(3) w(2);
      w(3) 0 -w(1);
     -w(2) w(1) 0];
end