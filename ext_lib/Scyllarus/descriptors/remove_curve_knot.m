%function [T, NEW_KNOTS, NEW_CP_REFLEC, NEW_CP_WAVE] = 
%    remove_curve_knot(degree, KNOTS, CP_REFLEC, CP_WAVE, index, mul, num, tolerance)
%   This function is designed according to Algorithm A5.8 of the NURBS book (Page 185). It decides
%   whether a knot is removable and how many times. 
%
% Syntax: 
%   [T, NEW_KNOTS, NEW_CP_REFLEC, NEW_CP_WAVE] = ...
%           remove_curve_knot(degree, KNOTS, CP_REFLEC, CP_WAVE, index, mul, num, tolerance);
%   Input:
%
%       degree:       degree of the curve.
%
%       KNOTS:     knot vector (row vector).
%
%       CP_REFLEC: control point coordinates in the reflectance dimension (stored as a height x
%                  width x (n + 1) 3D array while n + 1 is the number of control points per spectrum).
%
%       CP_WAVE:   control point coordinates in the wavelength dimension (stored as a (n + 1) x 1
%                  column vector, where n + 1 is the number of control points per spectrum). These
%                  control points are all the same for all the pixels. 
%
%       index:     index of the knot (in the knot vector) to be removed (index starts at 1).
%
%       mul:       the multiplicity of the knot to be removed.
%
%       num:       number of times intended to remove the knot.
%
%       tolerance: tolerance of the new control points displacement, in terms of distance.
%
%   Output:
%
%       T:             the number of times the indicated knot can be removed.
%
%       NEW_KNOTS:     the new knot vector after removing the knot T times.
%
%       NEW_CP_REFLEC: the new control point coordinates in the reflectance dimension after removing
%                      the knot T times, stored as a 3D array. 
%
%       NEW_CP_WAVE:   the new control point coordinates in the wavelength dimension after removing
%                      the knot T times, stored as a column vector. 
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei

function [T, NEW_KNOTS, NEW_CP_REFLEC, NEW_CP_WAVE] = remove_curve_knot(degree, KNOTS, CP_REFLEC, CP_WAVE, index, mul, num, tolerance)

    [T, NEW_KNOTS, NEW_CP_REFLEC, NEW_CP_WAVE] = remove_curve_knot_(degree, KNOTS, CP_REFLEC, CP_WAVE, index, mul, num, tolerance);
        
end