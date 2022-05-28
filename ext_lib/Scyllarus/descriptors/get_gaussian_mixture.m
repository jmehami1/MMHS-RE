%% Express the image spectra as a mixture of Gaussians
% 
%% Syntax
%   [MIXCOEFF, MEAN, STD] = get_gaussian_mixture(I, WAVELENGTH, M, DEBUG)
%
%% Description
%   This function computes the Gaussian mixture that represents the input spectra.
%
%   The method used in this function is the EM algorithm for fitting. At each 
%   iteration it re-estimate the components of the spectra at all pixels together. 
%   The features are the mixture coefficients, the mean and standard deviation of
%   the Gaussian components in the wavelength domain.
%
%% Input:
%
%       I: The 3D data matrix. (height x width x band)
%       WAVELENGTH: Wavelengths at which the image was captured.
%       M: The number of Gaussian components.
%       DEBUG: The level of debugging information to be displayed. Default to 1. Max is 3
%
%% Output:
%
%       MIXCOEFF: Mixture coefficients of the Gaussian components (height x width x M).
%       MEAN: The means of the Gaussian components (height x width x M).
%       STD: The standard deviations of the Gaussian components (height x width x M).
%
%% Example
%
%       Fit a mixture of 3 Gaussians to the spectra corresponding to the
%       illuminant on the Scyllarus data structure HSZ.
%
%       [mats, ~] = size(HSPipeline.L.Elements);
%       [MIXCOEFF, MEAN, STD] = get_gaussian_mixture(reshape(HSZ.L.Elements, [mats 1 bands]), HSZ.HDR.wavelength, 3);
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei, Cong Phuoc Huynh and Antonio Robles-Kelly            

function [MIXCOEFF, MEAN, STD] = get_gaussian_mixture(I, WAVELENGTH, M, DEBUG)

    switch nargin
        case 4
            [MIXCOEFF, MEAN, STD] = get_gaussian_mixture_(I, WAVELENGTH, M, DEBUG)
        case 3
            [MIXCOEFF, MEAN, STD] = get_gaussian_mixture_(I, WAVELENGTH, M)
        otherwise
            error('Please check input arguments');
    end

end