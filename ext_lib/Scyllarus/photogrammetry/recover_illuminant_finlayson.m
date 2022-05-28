% Syntax:
%        L = recover_illuminant_finlayson(I, PATCHES, MASK, debug)
%
% Estimate the light spectrum using the dichromatic plane + optimisation
% method proposed in the following paper
% 
% Graham D. Finlayson, Gerald Schaefer: 
% Convex and Non-convex Illuminant Constraints for Dichromatic Colour Constancy. 
% CVPR (1) 2001.
% 
% Input:
%
%   I:       height x width x bands, the input spectral radiance image.
%   PATCHES: selected patches over the given data cube
%   MASK:    masking matrix, if not given, it equals a no masking
%   debug:   Whether to show debug information during processing. 1 for yes
%            and 0 for no
% 
% Output:
%   
%   L: the estimate of the light spectrum.
%
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei, Antonio Robles-Kelly and Cong Phuoc Huynh

function L = recover_illuminant_finlayson(I, PATCHES, MASK, debug)

    switch nargin
        case 4
            L = recover_illuminant_finlayson_(I, PATCHES, MASK, debug);
        case 3
            L = recover_illuminant_finlayson_(I, PATCHES, MASK);
        case 2
            L = recover_illuminant_finlayson_(I, PATCHES);
        otherwise
            error('Incorrect input arguments');
    end
    
end