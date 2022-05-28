% Description
%   function index = findspan(n, degree, point, KNOTS)
%   This is the function to find knot span index of the parametric point point for computation used to
%   get basis functions for B-Spline. Actually, what this function does is to find the position for
%   the input value point in the knot vector so that at the returned position index, point >= KNOTS(index) and point
%   < KNOTS(index+1). This algorithm is described as `Algorithm A2.1' in `the NURBS Book' (Page 68).
%
% INPUT:
%       n:      the number of control points
%       degree: spline degree
%       point:  parametric point
%       KNOTS:  knot vector [1 x (m + 1)] (m = n + degree + 1).
%
% OUTPUT:
%
%       index:  knot span index (between degree+1 and n + 1)
%
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2015 All Rights Reserved
% Author: Ran Wei and Cong Phuoc Huynh



function index = findspan(n, degree, point, KNOTS)

    index = findspan_(n, degree, point, KNOTS);
    
end
