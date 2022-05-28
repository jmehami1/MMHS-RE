% Resize an HSZ or hyperspectral image
%
% Syntax:
%     HSZ = resize_image(HS, rows, cols);
%     HSZ = resize_image(HS, scale);
%     I   = resize_image(Im, rows, cols);
%     I   = resize_image(Im, scale);
%
% Description:
%     Resizes an HSZ or hyperspectral image.
% 
% Input:
%     I:            Image data structure
%     HS:           Scyllarus hyperspectral data structure
%     rows, cols:   New image cube dimensions
%     scale:        Scale up to which the image is to be resized.
% 
% Output:
%     I: Resized image data structure.
%     HSZ: Resized Scyllarus data structure.
%
% See also
%
%     crop_I, crop_image, resize_I
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function Q = resize_image(I, rows, cols)
    if nargin < 2
        error('Not enough input arguments');
    end
    
    if ~exist('cols', 'var')
        Q = resize_image_(I, rows);
    else
        Q = resize_image_(I, rows, cols);
    end
    
end
