%   This is the function of Global interpolation through all the reflectance spectra of the given
%   hyperspectral image (denoted by R in the input parameter list). This function was adapted from
%   Algorithm A9.1 on page 369-370 of "The NURBS book". 
%   
%   Function [KNOTS, EVA_PT, CP_REFLEC, CP_WAVE] = global_interpolation_(R, WAVELENGTHS, p, debug)
%
%   Input:
%       R:           the hyperspectral reflectance (or radiance) image (height x width x n). 
%       WAVELENGTHS: the wavelengths at which R was captured.
%       deg:         the degree of the B-spline basis function (by default: 3).
%
%   Output:
%       KNOTS:     knot vector (a column vector of size m x 1), same for all pixels.

%       EVA_PT:    parametric evaluation points corresponding to the bands (same for all pixels).
%   
%       CP_REFLEC: coordinates of control points in reflectance dimension, where CP_REFLEC(h, w, :)
%                  are the control point coordinates across all the bands at pixel coordinate (h,
%                  w). It is of the size: height x width x n 
%
%       CP_WAVE:   coordinates of control points in wavelength dimension (this is a 1 x n row
%                  vector). These coordinates in this dimension are the same for all the pixels in
%                  the given image.  
%   
%   Notes: 
%
%       All the reflectance spectrum (bands) correspond to the same set of bands indepedent
%       parameters (in the parameter domain). These parameters is computed by averaging the
%       independent parameter sets resulting from each individual spectrum. In this implementation,
%       the parameters are computed from the original distribution of the sampled
%       reflectance-wavelength pairs in each spectrum using the centripetal method of (Lee89)   
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014-2015 All Rights Reserved.
% Author: Ran Wei and Lin Gu


function [KNOTS, EVA_PT, CP_REFLEC, CP_WAVELT] = global_interpolation(R, WAVELENGTHS, deg, debug)

    switch nargin
        case 4
            [KNOTS, EVA_PT, CP_REFLEC, CP_WAVELT] = global_interpolation_(R, WAVELENGTHS, deg, debug);
        case 3
            [KNOTS, EVA_PT, CP_REFLEC, CP_WAVELT] = global_interpolation_(R, WAVELENGTHS, deg);
        case 2
            [KNOTS, EVA_PT, CP_REFLEC, CP_WAVELT] = global_interpolation_(R, WAVELENGTHS);
        otherwise
            error('Incorrect input arguments');
    end    
    
end