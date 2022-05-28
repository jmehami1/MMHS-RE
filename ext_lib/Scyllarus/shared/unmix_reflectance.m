% Reflectance unmixing routine
%
% Syntax:
%   HSZ = unmix_reflectance(HS, Endmembers);
%   HSZ = unmix_reflectance(HS, Endmembers, options);
% 
% Description:
%   This function is used to unmixe the materials in an HSZ data structure so as to 
%   refer to the library in Endmembers. If the HSZ is non-indexed, it converts 
%   it into an indexed file.
% 
% Input:
%     HS: Input Scyllarus data structure
%     options: Struct containing the following fields
%       Endmembers: Input struct containing the library
%       numEndmembers: Number of endmembers used for unmixing each material
%       PSFFactor: Factor used for the point spread function. This is
%               applied to enforce smoothness on the coefficients recovered by the
%               L-2 unmixing method.
% 
% Output:
%     HSZ:  Scyllarus data structure which has been indexed and unmixed to 
%        the endmember library. This is RAW encoded. For encodings other than 
%        RAW, use the encode_HSZ routine.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function HSZ = unmix_reflectance(HSPipeline, Endmembers, options)

    if ~exist('options', 'var')
        HSZ = unmix_reflectance_(HSPipeline, Endmembers);
    else
        HSZ = unmix_reflectance_(HSPipeline, Endmembers, options);
    end

end