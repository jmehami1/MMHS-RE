% Recover the reflectance from an HSZ data structure
%
% Syntax:
%     R = reconstruct_reflectance(HSZ);
% 
% Description:
%     Recover the reflectance from an HSZ file 
% 
% Input:
%     HSZ: HSZ structure
%     
% Output:
%     R: 3D array corresponding to the reflectance
%
% See also:
%    reconstruct_illuminant
% 
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function R = reconstruct_reflectance(HSZ)
   
    R = reconstruct_reflectance_(HSZ);
    
end