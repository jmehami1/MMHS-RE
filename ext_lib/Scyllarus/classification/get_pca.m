% Use principal Components Analysis (PCA) to a hyperspectral image file
%
% Syntax:
%      D = get_pca(I, n)
%
% Description:
%   This function is designed to conduct PCA on given a hyperspectral file.
%   It will return a data structure (the same as input I) which contains
%   the first n principal components generated from I. 
%
%   Inputs:
%
%       I: Structure containing the image cube and the header. Or, a
%          hyperspectral image cube of size height-by-width-by-band.
%       n: Number of principal components to be kept of PCA
%
%   Output:
%
%       D: The data structure containing n principal components. It shares
%          the same structure as I. D will has the same value range as I
%          does.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function D = get_pca(I, n)

    if nargin == 2
        D = get_pca_(I, n);
    elseif nargin == 1
        D = get_pca_(I);
    else
        error('Incorrect input arguments');
    end
    
end