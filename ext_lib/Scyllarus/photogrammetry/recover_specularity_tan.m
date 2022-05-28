%   Syntax:
%       K = recover_specularity_tan(I)
%   
%   Description:
%   Recover the specularity map from a single hyperspectral image using the method 
%   in
%         Robby T. Tan, Katsushi Ikeuchi, IEEE Transactions on Pattern Analysis and Machine 
%         and Machine Intelligence (PAMI), 27(2), pp.179-193, February, 2005
%      
%   Input: 
%       I: Input image cube, can be in either multi-spectrum or RGB image
%
%   Output: 
%
%   k: the map of specular coefficients at the image pixels, stored as a 2D array of size height x
%      width.  
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei

function K = recover_specularity_tan(I)

    K = TanRGBDespec(I);

end

