%   Function D = get_2d_gaussian_derivative_(I2D, sigma, xorder, yorder)
%   
%   is to get Gaussian derivative of a given 2D matrix using different
%   settings of sigma, and order
%   
%   Input:
%       I2D:    The input matrix
%       sigma:  value of sigma, default to 1; Sigma can not be 0.
%       xorder: order on x directrion, values between [0, 2], default to 0
%       yorder: order on y direction, values between [0, 2], default to 0
%
%   Output:
%       D:      The deriviatives of I2D
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2015 All Rights Reserved.
% Author: Ran Wei

function D = get_2d_gaussian_derivative(I2D, sigma, xorder, yorder)
    
    switch nargin
        case 4
            D = get_2d_gaussian_derivative_(I2D, sigma, xorder, yorder);
        case 3
            D = get_2d_gaussian_derivative_(I2D, sigma, xorder);
        case 2
            D = get_2d_gaussian_derivative_(I2D, sigma);
        case 1
            D = get_2d_gaussian_derivative_(I2D);
        otherwise
            error('Incorrect input arguments');
    end

end