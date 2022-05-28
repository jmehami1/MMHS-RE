% Reconstruct a FLA data structure from a HSZ structure
%
% Syntax:
%
%     I = reconstruct_image(HSZ);
% 
% Description:
%     Recover a hyperspectral image structure (the image cube and the header) 
%     from an HSZ structure which can be imported from HSZ files by calling 
%     function HSZread 
% 
% Input:
%     HSZ:  HSZ structure
%     
% Output:
%     I:    Structure containing the image cube (I.I) and the corresponding header (I.HDR).
% 
% See also
%
%     Scyllarus
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function I = reconstruct_image(HSZ)

    I = reconstruct_image_(HSZ);
    
end
    