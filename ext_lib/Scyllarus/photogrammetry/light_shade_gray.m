% Shade of Gray method for estimating the illuminant colour from an image 
% by taking the L^p mean of the image brightness in each channel, where p 
% is an order taken as input.
%
% Usage:
%   L = light_shade_gray(I, drate, order)
% 
% Input: 
%   I:     a spectral (or RGB) image with size height x width x band
%   drate: the downsampling rate.
%   order: the order p of the L^p mean. By default, order = 2;
%
% Output: 
%   L: the illumination power spectrum with size band-by-1
% 
%% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.

function L = light_shade_gray(I, drate, order)
    
    if nargin == 3
        L = light_shade_gray_(I, drate, order);
    elseif nargin == 2
        L = light_shade_gray_(I, drate);
    else
        error('Incorrect input arguments');
    end
    
end