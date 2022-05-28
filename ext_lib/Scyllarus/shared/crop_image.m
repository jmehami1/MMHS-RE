% Crop an HSZ or hyperspectral image
%
% Syntax:
%     HSZ = crop_image(HS, rect);
%     I = crop_image(Im, rect);
%
% Description:
%     Crops an HSZ or hyperspectral image.
% 
% Input:
%     I: Image data structure
%     HS: Scyllarus hyperspectral data structure
%     rect: rect is a four-element position vector[xmin ymin width height] 
%           that specifies the size and position of the crop rectangle. 
% 
% Output:
%     I: Cropped image data structure.
%     HSZ: Cropped Scyllarus data structure.
%
% Example
%   
%     Crop an image based on a rect of width 150 and height of 200 pixels.
%
%   I = FLAread('.\shared\samples\face.fla');
%   Q = crop_image(I.I, [80 70 150 200]);
%
% See also
%
%     crop_I, resize_image, crop_I
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function Q = crop_image(I, rect)

    if nargin < 2
        error('Not enough input arguments');
    end
    
    Q = crop_image_(I, rect);
    
end
