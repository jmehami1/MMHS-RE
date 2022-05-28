% [REFLEC_CPS, WAVE_CPS] = get_deri_cps(BSP_DEG, KNOTS, CP_REFLEC, CP_WAVE, MAX_DEG, MIN_CP_INDEX, MAX_CP_INDEX)   
%
%   This function Compute the control points of all the derivatives of a hyperspectral image
%   reflectance and wavelength with respect to the independent parameter, with order up to and
%   including the MAX_DEGree^{th} derivative. 
%   
% Input:
%       BSP_DEG: B-spline degree.
%       
%       KNOTS: knot sequence, vector [1 x (m + 1)] (m = n + BSP_DEG + 1), 
%           where n is the number of control points - 1.
%           
%       CP_REFLEC: (height x width x nCtrls) the control point coordinates
%           in the reflectance dimension, where CP_REFLEC(h, w, :) are the 
%           control point coordinates across all the bands 
%           at pixel coordinate (h, w).
%   
%       CP_WAVE: (nCtrls x 1) - a row vector - the control point coordinates
%           in the wavelength dimension. These coordinates in this dimension
%           is the same for all the pixels in the given image.
%           nCtrls is the number of control points.    
%   
%       MAX_DEG: the maximum degree of the derivatives.
%       
%       MIN_CP_INDEX, MAX_CP_INDEX: the lower bound and upper bound of the
%           index of the control points in the original curves 
%           which are then used to generate control points of the
%           derivatives of higher orders according to the formula in
%           Equation 3.8 (1 <= MIN_CP_INDEX <= MAX_CP_INDEX <= n)
%           (since Matlab arrays are indexed from 1)
%              
% Output:
%
%       REFLEC_CPS: the reflectance (or radiance) 
%           coordinates of the control points of 
%           the derivative curves (which are also B-Splines), 
%           with dimension 
%           (height x width x (MAX_CP_INDEX - MIN_CP_INDEX + 1) x MAX_DEGree),       
%           where REFLEC_CPS(i, j, k, l) is reflectance (or radiance)
%           coordinate of the control points of the derivative curve of
%           order l(1 <= l <= BSP_DEG), 
%           for the pixel (i, j) and the k^th wavelength.
%       
%       WAVE_CPS: : the wavelength coordinates of the control points 
%           of the derivative curves (which are also B-Splines), 
%           with dimension ((MAX_CP_INDEX - MIN_CP_INDEX + 1) x MAX_DEGree), 
%           where WAVE_CPS(k, l) is wavelength
%           coordinate of the control points of the derivative curve 
%           of order l(1 <= l <= BSP_DEG) for the k^th sampled wavelength.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei

function [REFLEC_CPS, WAVE_CPS] = get_deri_cps(BSP_DEG, KNOTS, CP_REFLEC, CP_WAVE, MAX_DEG, MIN_CP_INDEX, MAX_CP_INDEX)

    [REFLEC_CPS, WAVE_CPS] = get_deri_cps_(BSP_DEG, KNOTS, CP_REFLEC, CP_WAVE, MAX_DEG, MIN_CP_INDEX, MAX_CP_INDEX);
    
end