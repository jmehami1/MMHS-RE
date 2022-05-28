% Evaluate the specular highlights from an HSZ image file
%
% Syntax:
% K = reconstruct_specularity(HSZ);
% 
% Description:
% Recover the specularity from a HSZ structure
% 
% Input:
% HSZ: HSZ structure
%     
% Output:
% K: 3D array corresponding to the specularities
%
% See also:
%
% Scyllarus
%
% Scyllarus, reconstruct_illuminant, reconstruct_reflectance
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.

function K = reconstruct_specularity(HSZ)

    K = reconstruct_specularity_(HSZ);
    
end
