%   Function POINTS = univar_bspline_(deg, CTR_PTS, KNOTS, PARAMS)
% 
%   This function is designed to- Evaluate a univariate B-Spline.
%    (Algorithm 3.1 in the NURBS book).
% 
% Description:
% 
%   Evaluate a univariate B-Spline. This function provides an interface to
%   a toolbox 'C' routine.
%
% Parameters:
% 
%   INPUT:
%
%       deg:     Degree of the B-Spline.
% 
%       CTR_PTS: Control Points, matrix of size n by dim. Mostly it
%                should be a column vector.
% 
%       KNOTS:   Knot sequence, row vector of size nKnots.
% 
%       PARAMS:  Parametric evaluation points, row vector of size params_n.
% 
%   OUTPUT:
%
%       POINTS:  Evaluated points, matrix of size (params_n, dim)
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2014 All Rights Reserved.
% Author: Ran Wei

function POINTS = univar_bspline(deg, CTR_PTS, KNOTS, PARAMS)

    POINTS = univar_bspline_(deg, CTR_PTS, KNOTS, PARAMS);

end

