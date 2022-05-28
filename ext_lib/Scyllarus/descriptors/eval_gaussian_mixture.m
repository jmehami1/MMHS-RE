%% Syntax:
%   R = eval_gaussian_mixture(MIXCOEFF, MEAN, STD, WAVELENGTH)
%
%% Description
%
%   This function is used to reconstruct the hyperspectral spectrum from Gaussian descriptors. 
%   These descriptors include the mixing coefficients, mean and standard deviations 
%   of the Gaussian mixture as fitted to each spectrum.
%
%%   Input:
%
%       MIXCOEFF, MEAN, STD: Mixture coefficients, means and standard deviations 
%           of size height x width x M.
%       WAVELENGTH: the wavelength vector at which the reconstructed reflectance
%
%%   Output:
%
%       R: the reconstructed image cube, of size (height x width x band), 
%          where band is the number of wavelengths in WAVELENGTH.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei, Cong Phuoc Huynh and Antonio Robles Kelly.

function R = eval_gaussian_mixture(MIXCOEFF, MEAN, STD, WAVELENGTH)

    R = eval_gaussian_mixture_(MIXCOEFF, MEAN, STD, WAVELENGTH);

end