% Recover a single illuminant from a hyperspectral image
%
% Syntax
%   L = recover_global_illuminant(I)
%   L = recover_global_illuminant(I, options)
%   L = recover_global_illuminant(I, options, DEBUG)
% 
% Description:
%   Recover a signle illuminant from a single hyperspectral image. 
%
% Input:
%
%   I: hyperspectral image stored as a 3D array.
%   options: Structure with the following fields
%           bitdepth: Is the data type for the spectral cube, i.e. number of bits per
%               spectral measurement. By fault this is 16.
%           method: Selects between the following methods 
%               'HRK': Employs the method of Huynh and Robles-Kelly (A Solution of the 
%                   Dichromatic Model for Multispectral Photometric Invariance, International 
%                   Journal of Computer Vision 2010).
%               'FS': Uses the method of Finlayson and Schaefer (Convex and Non-convex Illuminant 
%                   Constraints for Dichromatic Colour Constancy, CVPR 2001).
%               'GW': Uses the Grey World method.
%               'SG': Uses the Shade of Grey method.
%               'WP': Uses the White Patch method.
%               '1stOGE': Uses the 1st order Grey Edge method.
%               '2ndOGE': Uses the 2nd order Grey Edge method.
%           drate: Image downsampling rate for the Grey World, Shade of Grey,
%                   White Patch and Grey Edge methods. The default is 1,
%                   i.e. no downsampling.
%           shadeOfGreyOrder: The order of the L^p mean used for the Shade of Grey method.
%                   The default is 1.
%           alpha:   The value for the regularisation term used for the HRK
%               (Huynh and Robles-Kelly) method. The default for this is 50. 
%           patches: Pre-selected patches. This could be a set of geometry data of patches
%               with a format of (Top_left_y, top_left_x, height,
%               width). This can be left empty.
%           DEBUG: Defines the level of debugging information shown at execusion time (DEBUG<6).
%               the default is 0. 
% Output:
%
%   L: a 2D array of size (1 x bands), where bands is the number of wavelength
%           indexed bands in the input image.
% See also:
%   recover_multi_illuminant
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly. 

function L = recover_global_illuminant(I, options)

    switch nargin
        case 2
            L = recover_global_illuminant_(I, options);
        case 1
            L = recover_global_illuminant_(I);
        otherwise
            error('Incorrect input arguments');
    end

end

  