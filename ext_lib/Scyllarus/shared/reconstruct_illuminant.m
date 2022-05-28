% Illuminant evaluation routine for HSZ data structures
%
% Syntax:
%     L = reconstruct_illuminant(HSZ);
% 
% Description:
%     Recover the illuminant from an HSZ structure 
% 
% Input:
%     HSZ: HSZ structure (see NICTApipeline help for more details)
%     
% Output:
%     L: 3D array corresponding to the illuminant
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.

function L = reconstruct_illuminant(HSZ)

    L = reconstruct_illuminant_(HSZ);

end