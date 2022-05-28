% Syntax:
%   G = reconstruct_shading(HSZ);
% 
% Description:
%   Returns the shading as stored on the HSZ structure
% 
% Input:
%   HSZ: NICTA pipeline structure
% 
% Output:
%   G: Shading factor
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function G = reconstruct_shading(HSZ)

    G = reconstruct_shading_(HSZ);
    
end