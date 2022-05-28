% Illuminant unmixing routine
%
% Syntax:
%   HSZ = unmix_illuminant(HS, Endmembers);
%   HSZ = unmix_illuminant(HS, Endmembers, options);
% 
% Description:
%     Unmixes the illuminants in an HSZ file so as to refer to the library in 
%     Endmembers. If the HSZ is non-indexed, it converts it into an indexed file.
% 
% Input:
%     HS: Input Scyllarus data structure
%     options: Struct containing the following fields
%           numCanonicalIlluminants.: Number of endmembers used for unmixing each material
%           numIlluminants: Number of illuminants used for the indexing of the
%                   light in the image.
%           PSFFactor: Factor used for the point spread function. This is
%                   applied to enforce smoothness on the coefficients recovered by the
%                   L-2 unmixing method.
%
% Output:
%     HSZ: Scyllarus data structure indexed and unmixed to the endmember 
%         library. This is RAW encoded. For encodings other than RAW, use the
%         encode_HSZ routine.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function HSZ = unmix_illuminant(HSPipeline, CanonicalIlluminants, options)
    
    if ~exist('options', 'var')
        HSZ = unmix_illuminant_(HSPipeline, CanonicalIlluminants);
    else
        HSZ = unmix_illuminant_(HSPipeline, CanonicalIlluminants, options);
    end
    
end