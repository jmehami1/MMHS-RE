%     function [MIN_KNOTS, MIN_CP_REFLEC, MIN_CP_WAVE] = ...
%            get_minimal_knots(I, WAVELENGTHS, degree, KNOTS, CP_REFLEC, CP_WAVE, alpha, target_knotnum, debug)
%
%   This function is the implementation of the knot removal algorithm for a hyperspectral image.
%   It's coded according to the `Algorithm 1' in the paper `A B-Spline Representation of Spectral
%   Images' by Cong Phuoc Huynh and Antonio Robles-Kelly.
%
%   Input:
%
%       I:              the input set reflectance image - a 3D array of size (height x width x bands).
%
%       WAVELENGTHS:    the wavelengths of the multispectral image.
%
%       degree:         the degree of the basis functions.
%
%       KNOTS:          the original knot vector (in the form of a row vector).
%
%       CP_REFLEC:      the initial coordinates of control points in the reflectance dimension,
%                       where P_ref(h, w, :) are the control point coordinates across all the bands
%                       at pixel coordinate (h, w). It is of size height x width x bands. Using
%                       function global_interpolation.m can generate such control points.
%
%       CP_WAVE:        a 1 x bands row vector - the initial control point coordinates in the
%                       wavelength dimension. These coordinates in this dimension is the same for
%                       all the pixels in the given image. Using global_interpolation.m can generate
%                       such control points.
%
%       alpha:          balance factor between 0 and 1.
%
%       target_knotnum: the target number of KNOTS remained.
%       
%       debug:          The level of debugging information. 0 for none and 1 for all
%
%       tolerance:      tolerance on the control point displacement. If not set, it will be set to INF to allow as many knots to be removed as specified.
%
%    Output:
%
%       MIN_KNOTS:      the minimal knot vector, in the form of a row vector.
%
%       MIN_CP_REFLEC:  the minimal control point coordinates in the reflectance dimension - in the
%                       form of a 3D array.
%
%       MIN_CP_WAVE:    the minimal control point coordinates in the wavelength dimension - in the
%                       form of a row vector.
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2015 All Rights Reserved.
% Author: Ran Wei

function [MIN_KNOTS, MIN_CP_REFLEC, MIN_CP_WAVE] = ...
    get_minimal_knots(I, WAVELENGTHS, degree, KNOTS, CP_REFLEC, CP_WAVE, alpha, target_knotnum, tolerance, debug)

    switch nargin
        case 10
            [MIN_KNOTS, MIN_CP_REFLEC, MIN_CP_WAVE] = ...
            get_minimal_knots_(I, WAVELENGTHS, degree, KNOTS, CP_REFLEC, CP_WAVE, alpha, target_knotnum, tolerance, debug);
        case 9
            [MIN_KNOTS, MIN_CP_REFLEC, MIN_CP_WAVE] = ...
            get_minimal_knots_(I, WAVELENGTHS, degree, KNOTS, CP_REFLEC, CP_WAVE, alpha, target_knotnum, tolerance);
        case 8
            [MIN_KNOTS, MIN_CP_REFLEC, MIN_CP_WAVE] = ...
            get_minimal_knots_(I, WAVELENGTHS, degree, KNOTS, CP_REFLEC, CP_WAVE, alpha, target_knotnum);
        otherwise
            error('Incorrect input arguments');
    end    
    
end