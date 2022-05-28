% Compute the NURBS representation for spectral data
%
% Syntax
%   [KNOTS, CP_REF, CP_WAVE] = get_nurbs(R, WAVES, degree, knot_thresh, alpha, iter)
%
% Description
%
%       This function is designed to a generate NURBS representation for a 
%       given set of hyperspectral data.
%
% Input:
%
%       R:           Input spectral data, stored as a 3D array of size (height x width x bands).
%       WAVES:       Wavelength vector for the spectra.
%       degree:      Degree of the basis (polynominal) functions. By default degree = 3;
%       alpha:       A balancing factor between 0 and 1, which is the weight of the data 
%                    closeness term (reconstruction error) 
%                    with respect to the original data. By default alpha = 0.1
%       knot_thresh: The threshold for the number of minimal knots. By default 
%                    knot_thresh = band - 2;
%       iter:        the maximum number of iterations. By default iter = 10
%       debug:       Whether to show debugging information. 1 or 2 for yes and 0 for no (default)
%   
% Output:
%
%       KNOTS:   Final minimal knot vector - row column.
%       CP_REF:  Minimal control point coordinates in the reflectance
%                dimension - in the form of a 3D array of size (height x width x ctrl_pts).
%       CP_WAVE: Minimal control point coordinates in the wavelength
%                dimension - in the form of a row vector (1 x ctrl_pts).
%
% Example
%
%   [KNOTS, CP_REF, CP_WAVE] = get_nurbs(HSZ.S.Elements, HSZ.HDR.wavelength, 2);
%            
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function [KNOTS, CP_REF, CP_WAVE] = get_nurbs(R, WAVES, degree, knot_thresh, alpha, iter, debug)

    n = length(WAVES);
    WAVES = reshape(WAVES, [n 1]);
    switch nargin
        case 7
            [KNOTS, CP_REF, CP_WAVE] = get_nurbs_(R, WAVES, degree, knot_thresh, alpha, iter, debug);
        case 6
            [KNOTS, CP_REF, CP_WAVE] = get_nurbs_(R, WAVES, degree, knot_thresh, alpha, iter);
        case 5
            [KNOTS, CP_REF, CP_WAVE] = get_nurbs_(R, WAVES, degree, knot_thresh, alpha);
        case 4
            [KNOTS, CP_REF, CP_WAVE] = get_nurbs_(R, WAVES, degree, knot_thresh);
        case 3
            [KNOTS, CP_REF, CP_WAVE] = get_nurbs_(R, WAVES, degree);
        case 2
            [KNOTS, CP_REF, CP_WAVE] = get_nurbs_(R, WAVES);
        otherwise
            error('Incorrect input arguments');
    end     
    
end