%  Calculate norm between two input matrices
%
% Syntax
%   NORM = get_norm(M, N, form); or
%   NORM = get_norm(M, N);
%
% Description
%
%   Calculate the norm between two input matrices M and N in the form of L1 norm or L2 norm. M and N can be vectors, 
%   2D and 3D matrix as long as they are of same dimensions.
%    
% Input:
%   M and N: vectors, 2D or 3D matrix of source image
%   form:    'L1' or 'L2' or 'L1norm' or 'L2norm', for L1 norm and L2 norm accordingly, case insensitive. By default,
%            L2 norm is used;
%
% Output:
%   NORM:    norm value(s) between two input matrices. If M is a scalar or a vector, NORM will be a scalar. If M 
%            is a 2D matrix, NORM will be a row vector. If M is a 3D matrix of h by w by b, NORM will be a matrix of 
%            h by w.
%   
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function NORM = get_norm(M, N, form)

    %   check the number of inputs;
    if nargin < 2
        error('Not enough input arguments');
    end

    if ~exist('form', 'var')
        form = 'L2';
    end

    NORM = get_norm_(M, N, form);
    
end