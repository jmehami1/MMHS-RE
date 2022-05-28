% Description
%       N = get_basis_(span, point, deg, KNOTS)
%   Compute the non-zero basis function vectors at a parametric point point. 
% 
% INPUT:
%
%       span:  knot span ( from function findspan() )
%       point: parametric point
%       deg:   degree of the spline basis functions. Note: span should be
%              equal to or greater than deg. 
%       KNOTS: knot sequence (row vector containing [1 x (m + 1)] knots
%              where (m = n+1+deg), n + 1 is the number of control points).  
%
% OUTPUT:
%
%       N:      Non-zero basis functions vector with degree deg, 
%               evaluated at point, stored in an array of size [1 x (deg+1)].
%
% Explanation:
%
%       According to Property P2.1 (page 55 of "The NURBS book" 2nd edition), N_{span, deg}(point) = 0 if point
%       is outside the interval [u_{span}, u_{span+deg+1}) `.e. assuming that the knot span of point is 
%               s (u_{s} <= point < u_{s + 1}), 
%       then 
%               N_{span, deg}(point) = 0 if s < span or s >= span + deg + 1, 
%       So the algorithm below aims to compute N_{span, deg}(point) with span such that 
%               s - deg <= span <= s. 
%       The function is coded according to Algorithm A2.2 from 'The NURBS BOOK' (Page 70).
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2015 All Rights Reserved.
% Author: Ran Wei and Cong Phuoc Huynh

function N = get_basis(span, point, deg, KNOTS)

    N = get_basis_(span, point, deg, KNOTS);
    
end