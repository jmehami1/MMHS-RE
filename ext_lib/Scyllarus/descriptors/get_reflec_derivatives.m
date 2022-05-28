%   Function: DERI = get_reflec_derivatives(R, WAVELENGTH, deg, max_deg)
%   This function is designed to compute the derivatives of the reflectance or radiance (given as
%   the input variable R) with respect to the wavelength, using the B-Spline interpolation to the
%   spectrum at every pixel. The derivatives are computed at every sampled wavelength of the image
%
%   Parameters
% 
%   Input:
%
%       R:           hyperspectral reflectance (or radiance) image (height x width x bands).
%
%       WAVELENGTH: the wavelengths at which the hyperspectral image R was captured.
%
%       deg:         the degree of the B-spline basis functions used for interpolation.
%
%       max_deg:     the maximum degree of computed derivatives.
%
%
%   Output:
%
%       DERI: a 4D vector whose first three dimensions are the same as those of the input
%             hyperspectral image R, and the fourth dimension corresponds to the order of the
%             derivative. i.e. DERI(:, :, :, k) is the k^th derivative of the image at every given
%             wavelength. 
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function DERI = get_reflec_derivatives(R, WAVELENGTH, deg, max_deg)

    DERI = get_reflec_derivatives_(R, WAVELENGTH, deg, max_deg);

end