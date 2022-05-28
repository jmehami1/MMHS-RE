% Syntax:
%   deg = angle(L1, L2);
% 
% Description:
% This function computes the angle between two spectra S1 and S2. 
% It returns the value in degrees. Note that this is an overloaded function.
% 
% Input:
%   L1, L2: Vectors
% 
% Output:
%   deg: Euclidean angle, in degrees, between L1 and L2
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei

function deg = angle(L1, L2)
    deg = angle_(L1, L2);
end