% White patch method for estimating the illuminant colour from an image 
% by taking the maximal image brightness in each channel 
% as the illuminant spectral band value.
%
% Usage:
%   L = light_white_patch(I, drate)
%
% Input: 
%   I:     a spectral (or RGB) image cube with size height x width x band
%   drate: the downsampling rate.
% 
% Output: 
%   L:     the illumination power spectrum with size band x 1
% 
%% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.

function L = light_white_patch(I, drate)

    L = light_white_patch_(I, drate);
    
end