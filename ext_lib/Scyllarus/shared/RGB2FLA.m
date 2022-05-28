% Convert a trichromatic image into an LFA data structure
%
% Syntax:
%     I = rgb2fla(IMAGE);
% 
% Description
%   Converts an RGB image into an FLA (flat) image structure.
%
% Input:
%     IMAGE: RGB of gray-scale image contained in a cols x rows x 3 array.
%
% Output:
%     I: Structure containing the flat image. This is designed to be 
%        consistent with that delivered by FLAread. 
%             
% Example:
%   IMAGE = imread('test.jpg'); 
%   I     = RGB2FLA(IMAGE);
%   HSZ   = Scyllarus(I);
%
% See also:
%   Scyllarus, FLAwrite, FLAread
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Antonio Robles-Kelly. 

function I = rgb2fla(IMAGE)

    I = rgb2fla_(IMAGE);
    
end