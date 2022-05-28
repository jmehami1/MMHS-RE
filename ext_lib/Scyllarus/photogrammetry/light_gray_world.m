% GreyWorld method for estimating the illuminant colour from an image 
% by taking the mean image brightness in each channel 
% as the illuminant spectral band value.
%
%% Usage:
%   L = light_gray_world(I, drate)
%
%% Input: 
%   I:      a spectral (or RGB) image with size height x width x nBands
%   drate:  the downsampling rate.
% 
%% Output: 
%   L:      the illumination power spectrum with size nBands x 1
% 
%% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.

function L = light_gray_world(I, drate)

    L = light_gray_world_(I, drate);
    
end