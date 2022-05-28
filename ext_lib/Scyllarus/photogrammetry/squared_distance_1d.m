%  Calculate squared_distance between two input positions of two elements in 1D vector.
%
% Syntax
%   sd = squared_distance(p1, p2, height, width);
%
% Description
%
%   Get squared distance between two 3D points in neuron network which is denoted by two elements in 1D vectors 
%    
% Input:
%   p1 and p2: positions of two elements in 1D vectors. These elements are reshaped from a 3D neuron matrix.
%   height and width: 2D dimensions of of the 3D neuron matrix
%
% Output:
%   sd: squared distance between two points in 3D neuron network. These two points are converted from given positions.
%   
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function sd = squared_distance_1d(p1, p2, height, width)

    sd = squared_distance_1d_(p1, p2, height, width);
    
end
