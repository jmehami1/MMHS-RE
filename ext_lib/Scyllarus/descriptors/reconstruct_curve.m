
%   Description: 
%       [R, WAVELENGTH] = reconstruct_curve(KNOTS, T, REF_CP, WAVES_CP, deg)
% 
%   This function is designed to recontruct the original hyperspectral image or the original set of
%   spectra from a common knot vector (knots) for all the spectra, a common set of independent
%   parameter values (T) corresponding to the sampled reflectance-wavelength pairs in all the
%   spectra, and the given control point coordinates in the reflectance dimension(REF_CP) and the
%   wavelength dimension (WAVES_CP). (Based on Algorithm 3.1 in the NURBS book).
% 
%   Input:
%
%       KNOTS      : Knot sequence, row vector of size nKnots.
%
%       T          : Independent parametric evaluation points, stored in a row vector of size band, where 
%                    band is the number of bands to be reconstructed.
% 
%       REF_CP     : control point coordinates in the reflectance dimension, a 3D array of size (height x
%                    width x CP_NUM), where CP_NUM is the number of control points in the refletance
%                    (radiance) spectrum at each pixel. 
% 
%       WAVES_CP   : control point coordinates in the wavelength dimension, a column vector of size (band x 1).
%
%       deg        : Degree of the B-Spline.
%   
%   Output:
%
%       R          : the reconstructed image, of size (height x width x band)
%
%       WAVELENGTH : the wavelengths corresponding to the bands in the image.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013-2014 All Rights Reserved.
% Author: Ran Wei

function [R, WAVELENGTH] = reconstruct_curve(KNOTS, T, REF_CP, WAVES_CP, deg)

    [R, WAVELENGTH] = reconstruct_curve_(KNOTS, T, REF_CP, WAVES_CP, deg);
    
end