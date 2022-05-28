%   function [R_DERIVS, WAVE_DERIVS] = get_para_derivatives(R, waves, deg, max_deg)
%
%   Compute the derivatives of the reflectance or radiance and wavelengths with respect to the
%   independent parameters, using an B-Spline interpolation to the spectrum at every pixel.
% 
%   The derivatives are computed at the independent parameter values corresponding to every wavelength. 
% 
%   Input:
%
%       R:           hyperspectral reflectance (or radiance) image (height x width x bands).
%       WAVELENGTH:  the wavelengths at which the hyperspectral image R was captured.
%       deg:         degree of the B-spline basis functions used for interpolation.
%       max_deg:     maximum degree allowed for the derivatives computed.
%
%   Output:
%
%       R_DERIVS:    derivatives of the reflectance or radiance with respect to the independent
%                    parameter, Its sizes are: height x width x bands x degree where degree =
%                    min(max_deg, deg). 
%   
%       WAVE_DERIVS: the derivative of the wavelength with respect to the independent parameter,
%                    where degree = min(max_deg, deg). These derivatives are the same for all the
%                    pixels in the given image. Size: bands x degree
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function [R_DERIVS, WAVE_DERIVS] = get_para_derivatives(R, WAVELENGTH, deg, max_deg)

    [R_DERIVS, WAVE_DERIVS] = get_para_derivatives_(R, WAVELENGTH, deg, max_deg);

end