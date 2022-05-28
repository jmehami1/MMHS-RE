%   Function [R, WAVE_NEW] = eval_nurbs(KNOTS, WAVES, CP_REF, CP_WAVE, degree)
%
%   This function is designed to reconstruct the hyperspectral image using NURBS matrix 
%
%   Input:
%       KNOTS:   the final minimal knot vector - row column.
%       
%       CP_REF:  the minimal control point coordinates in the reflectance dimension - in the form of a 3D array of size
%                (height x width x ctrl_pts). 
% 
%       CP_WAVE: the minimal control point coordinates in the wavelength dimension - in the form of a row vector (1 x
%                ctrl_pts). 
%
%       degree:  the degree of the basis (polynominal) functions. by default degree = 3;
%
%       WAVES:   the wavelengths at the bands of the image.
% 
%   
%   Output:
%
%       R:         the input multispectral image, stored as a 3D array of size (height x width x band).
% 
%       WAVE_NEW:  the actual wavelength where the generated image is built on 
% 
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013-2014 All Rights Reserved.
% Author: Ran Wei



% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved

% Version 1.0.1
% Date: 21 Jan 2013

function [R, WAVE_NEW] = eval_nurbs(KNOTS, WAVES, CP_REF, CP_WAVE, degree)

    switch nargin
        case 5
            [R, WAVE_NEW] = eval_nurbs_(KNOTS, WAVES, CP_REF, CP_WAVE, degree);
        case 4
            [R, WAVE_NEW] = eval_nurbs_(KNOTS, WAVES, CP_REF, CP_WAVE);
        otherwise
            Error('Incorrect input arguments');
    end
        
%   end of function
end