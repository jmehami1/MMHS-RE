%   function POINTS = find_parapoints_(WAVES, error, degree, KNOTS, CP)
%
%   Find the independent parametric evaluation point at which the evaluation of a B-Spline curve
%   with the given degree (degree), knots (KNOTS) and control points (CP) does not differ from the
%   given vector of coordinates WAVES more than a threshold (error). The prior assumption is that x is a
%   monotonic function with respect to the parameter t.
%
%   Output:
%       POINTS:  the approximating parameter values that yield the given coordinates WAVES, stored
%                as a row vector. 
%
%   Input:
%   
%       WAVES:   the wavelength (a m x 1 column vector).
%       error:   error tolerance of the approximating values in X (abs(WAVES - X(i)) < error).
%       degree:  degree of the curve.
%       KNOTS:   the knot vector (in the form of a row (1 x m) vector).
%       CP:      control point coordinates (n_ x r) of the curve in the dimension of given WAVES.  
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Cong Phuoc Huynh

function POINTS = find_parapoints(WAVES, error, degree, KNOTS, CP)

    POINTS = find_parapoints_(WAVES, error, degree, KNOTS, CP);
    
end

