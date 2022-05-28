% L2 unmixing routine for spectral data
%
% Syntax:
%       [UNMIX_WEIGHT_MAP, UNMIX_INDEX_MAP, ENDMEMBER, WAVELENGTH] = L2_unmixing(S, S_WL, BASIS, BASIS_WL, num_endmember)
%
% Description:
%       This function is designed to unmix spectral images into combinations of reference spectra
%       or end-members. This routine uses NURBS to perform unmixing of spectra with respect to 
%       end-members with different spectral resolutions. Note that the value of input S and BASIS will be
%       squared-normalised to 1 per pixel.
%
% Input:
%
%     S:        Hyperspectral image data cube. This can be either a 3-D array 
%               whose third dimension is the wavelength domain or a 2D array 
%               whose second dimension is indexed to wavelength
%     S_WL:     Vector of wavelengths used for the evaluation of the spectra in S.
%     BASIS:    The input library of endmember spectra. This is a 2D array whose
%               second dimension is the wavelength domain.
%     BASIS_WL: Vector of wavelengths used for the evaluation of the end-member spectra.
%     num_endmember:   Number of end-members used for the unmixing operation.
%
% Output:
%
%     UNMIX_WEIGHT_MAP: The proportion of the end-member corresponding to every pixel in the input reflectance image,
%                       the index of BASIS is given in UNMIX_INDEX_MAP. It will be nomralised to 1.
%     UNMIX_INDEX_MAP:  The index of the end-member for UNMIX_WEIGHT_MAP
%     ENDMEMBER:        Matrix of endmembers evaluated over the common wavelength
%                       range for both, the image data and the library.
%     WAVELENGTH:       Vector of common wavelength values for the image data and
%                       the end-member library.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function [UNMIX_WEIGHT_MAP, UNMIX_INDEX_MAP, ENDMEMBER, WAVELENGTH] = L2_unmixing(S, S_WL, BASIS, BASIS_WL, num_endmember)
    
    switch nargin
        case 5
            [UNMIX_WEIGHT_MAP, UNMIX_INDEX_MAP, ENDMEMBER, WAVELENGTH] = L2_unmixing_(S, S_WL, BASIS, BASIS_WL, num_endmember);
        case 4
            [UNMIX_WEIGHT_MAP, UNMIX_INDEX_MAP, ENDMEMBER, WAVELENGTH] = L2_unmixing_(S, S_WL, BASIS, BASIS_WL);
        case 3
            [UNMIX_WEIGHT_MAP, UNMIX_INDEX_MAP, ENDMEMBER, WAVELENGTH] = L2_unmixing_(S, S_WL, BASIS);
        otherwise
            error('Insufficent input argument number');
    end

end