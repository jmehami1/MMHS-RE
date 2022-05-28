%   function N = get_basis_funcs(span, PARA_POINTS, deg, KNOTS)
%
%   This function is designed to compute all the non-zero basis functions of degrees from 0 to deg,
%   evaluated at a parametric point PARA_POINTS. This algorithm is invoked by Algorithm A3.4 in 'The
%   NURBS Book' Page 99.
% 
%   INPUT:
%
%       span:   knot span  ( Use findspan.m to get this )
%       PARA_POINTS: the parametric point at which basis functions are evaluated.
%       deg:         the maximum degree of the spline basis functions.
%   `   KNOTS:       knot sequence (vector containing [1 x (m + 1)] knots. where m = n + deg + 1 and
%                    n + 1 is the number of control points).
%
%   OUTPUT:
%
%       N:           A matrix of size (deg+1) x (deg+1) storing the values of the B-Spline basis
%                    functions, where the jth row corresponds to all the non-zero basis functions of
%                    the jth-degree evaluated at the parameter value PARA_POINTS. N(j, r) is the
%                    value of the r^th basis function of degree j - 1. 
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei

function N = get_basis_funcs(span, PARA_POINTS, deg, KNOTS)

    N = get_basis_funcs_(span, PARA_POINTS, deg, KNOTS);

end