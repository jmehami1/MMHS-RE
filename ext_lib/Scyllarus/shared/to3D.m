% this function was initially designed to convert a 1D illuminant vector
% L into a 3D form so that other functions may use it easier.
%
% Description:
%       function L3D = to3D(L, size3D)
%   
% Input:
%       L: the 1D vector or 2D matrix
%       size3D: the size of 3D matrix that L is going to be converted to
%
% Output:
%       L3D: 3D version of L with the 3D size specified in size. L's
%            elements will be repeated on band direction (3rd one in size3D).
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function L3D = to3D(L, size3D)

    L3D = to3D_(L, size3D);

end