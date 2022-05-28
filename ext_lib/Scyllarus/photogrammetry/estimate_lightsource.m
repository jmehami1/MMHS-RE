% Function to estimates the light source of an input_image. Depending on
% the parameters the estimation is equal to Grey-Wolrd, Max-RGB, general
% Grey-World, Shades-of-Gray or Grey-Edge algorithm. 
%
% SYNOPSIS
%   [L, C] = estimate_lightsource_(I, order, norm, MASK);
%
% INPUT :
%   I:      Hyperspectral image data cube
%	order:  the order of differentiation (ranges from 0 to 2). Default to
%           0. Users can also use method names, ie 'grey_world', 'maxrgb',
%           'shades_of_grey' or 'grey_edge' instead of actual number for
%           order. 
%	norm:   minkowski norm used (if norm ==  -1 means infinity). Default to 2
%   sigma:  value of sigma, default to 0
%   MASK:   Binary images with zeros on image borders should be ignored. If
%           MASK is missing, the default value will be generated according
%           to the value of order. 
% OUTPUT: 
%   L:      illuminant estimation
%   C:      light corrected image
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2015 All Rights Reserved.
% Author: Ran Wei

function [L, C] = estimate_lightsource(I, order, norm, sigma, MASK)
    switch nargin
        case 5
            [L, C] = estimate_lightsource_(I, order, norm, sigma, MASK);
        case 4
            [L, C] = estimate_lightsource_(I, order, norm, MASK);
        case 3
            [L, C] = estimate_lightsource_(I, order, norm);
        case 2
            [L, C] = estimate_lightsource_(I, order);
        case 1
            [L, C] = estimate_lightsource_(I);
        otherwise
            error('Incorrect input arguments');
    end
end    




