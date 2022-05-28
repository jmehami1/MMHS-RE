% Gray Edge method for estimating the illuminant colour from an image 
% by taking the Minkowski mean of the image gradient magnitudes 
% in each channel, where an order of the mean is taken as input.
%
% Usage:
%   L = light_gray_edge(I, drate, order)
% 
% Input: 
%   I:     a spectral (or RGB) image with size height x width x bands
%   drate: the downsampling rate.
%   order: the order of the Minkowski mean. Default order == 2;
% 
% Output: 
%   L:     the illumination power spectrum with size bands x 1
% 
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.

function L = light_gray_edge(I, drate, order)

    if nargin == 3
        L = light_gray_edge_(I, drate, order);
    elseif nargin == 2
        L = light_gray_edge_(I, drate);
    else
        error('Incorrect input arguments');
    end
    
    
end