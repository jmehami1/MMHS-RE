% Syntax:
%
%       L = recover_illuminant_huynh(I, alpha, PATCHES, debug)
%
% Input:
%
%       I:        hyperspectral image stored as a 3D array.
%       alpha:    the variable used to test the result. default to 50
%       PATCHES:  number of PATCHES used. default to 20;
%       debug:    Whether to show debug information during processing. 1 for yes and 0 for no (Default).
%
% Output:
%
%       L: the illuminate computed, which is a 2D array of size (nBands x 1), where nBands is the number of bands,
%          storinglight spectral power. 
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 - 2015 All Rights Reserved.
% Author: Ran Wei and Cong Phuoc Huynh

function L = recover_illuminant_huynh(I, alpha, PATCHES, debug)

    switch nargin
        case 4
            L = recover_illuminant_huynh_(I, alpha, PATCHES, debug);
        case 3
            L = recover_illuminant_huynh_(I, alpha, PATCHES);
        case 2
            L = recover_illuminant_huynh_(I, alpha);
        case 1
            L = recover_illuminant_huynh_(I);
        otherwise
            error('Incorrect input arguments');
    end
    
end

