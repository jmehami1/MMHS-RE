% Syntax:
%   [K, G, S] = recover_dichromatic_parameters_LS(I, L, debug, neighb_size, gray_threshold)
%   
%   Description:
%   This function removes specularities from a hyperspectral image. All the parameters below
%   are defined in the context of the dichromatic reflection model.
% 
% Input: 
% 
%   I: the radiance image (stored as a 3D array with size height x width x bands).
%   L: the illuminant power spectrum. It could be either a 1D vector of
%      size bands x 1
%
%   neighb_size (optional): The size of the neighbourhood used for the recovery of the
%       reflectance. The default value is neighb_size = 5;
%   gray_threshold (optional): a threshold used to determine whether a material is a shade of gray. If
%       the reflectance spectra within a cluster does not deviate from a uniform spectrum (a flat
%       line) beyond this threshold, then we will assume that it is a shade of gray and purely
%       diffuse. (default value is 2) gray_threshold = 2;
%   
%   debug: Defines the level of displaying debugging information. Default is 1, the least
%       information will be given
% 
% Output: 
%
%   K: the map of specular coefficients at the image pixels, stored as a 2D array of size height x
%      width.  
%   S: the normalised reflectance cube per pixel and wavelength, stored as a 3D array of size height
%      x width x bands. The reflectance spectrum at each pixel is normalised to a unit L2-norm. That
%      is the vector S(i, j, :) is normalised.
%       
%   G: the shading factor g per pixel. G is stored as a 2D array with a size of height x width. 
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.

function [K, G, S] = recover_dichromatic_parameters_LS(I, L, debug, neighb_size, gray_threshold)
    
    switch nargin
        case 5
            [K, G, S] = recover_dichromatic_parameters_LS_(I, L, debug, neighb_size, gray_threshold);
        case 4
            [K, G, S] = recover_dichromatic_parameters_LS_(I, L, debug, neighb_size);
        case 3
            [K, G, S] = recover_dichromatic_parameters_LS_(I, L, debug);
        case 2
            [K, G, S] = recover_dichromatic_parameters_LS_(I, L);
        otherwise
            error('Incorrect input arguments');
    end
    
end
