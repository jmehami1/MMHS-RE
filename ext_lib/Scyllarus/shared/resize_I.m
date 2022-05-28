% Resize a hyperspectral image cube
%
% Syntax:
%     Q = resize_I(I, scale);
%     Q = resize_I(I, rows, cols);
% 
% Description:
%     Resizes an image cube comprised of n wavelength indexed bands to be of
%     dimensions rows x cols x n.
% 
% Input:
%     I:          Image cube
%     scale:      Scaling factor for the image cube. This should be a value
%                 greater than 0, where values between 0 and 1 imply the
%                 output image cube is smaller in size and values greater
%                 than one yield a larger image cube.
%     rows, cols: New image cube dimensions
% 
% Output:
%     Q: Resized image cube.
%
% Example
%   
%     Resize an image cube so as to be 256 by 320 pixels.
%
%   I = FLAread('.\shared\samples\face.fla');
%   Q = resize_I(I.I, 256, 320);
%	or
%	Q = resize_I(I.I, 0.6);
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function Q = resize_I(I, rows, cols)
    
    if exist('cols', 'var')
        Q = resize_I_(I, rows, cols);
    else
        Q = resize_I_(I, rows);
    end

end
