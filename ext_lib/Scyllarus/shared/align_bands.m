%
% Function: 
%   I_aligned = align_bands(I, max_shift, window_size, crop_orig)
%
% Align all the band images of a hyperspectral image if there are misalignments. This function was originally developed
% for images taken by Fluxdata 3CCD 7Channel Cameras. These cameras can take hyperspectral images of 7 bands. Among these
% bands, there is certain random misalignment which may be resulted from CCD alignment error during manufacturing.
% 
% Input: 
%   I:           an input hyperspectral image.
%   max_shift:   maximal shift in pixel in both directions. Default to 20
%   window_size: the size of the neighborhood used to compute the focus value of every pixel. If window_size = 0, a
%                single focus measure is computed for the whole image. Default to 3.
%   crop_orig:   Whether to crop output image to cut off misalignment edges. Default to 1 meaning: yes
% Output: 
%   I_aligned:   a new hyperspectral image whose pixels in different band are aligned. If crop_orig == 1, depend on the
%                misalignment size, I_aligned will be cropped to smaller than original dimension in height and width
% 
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Cong Phuoc Huynh and Ran Wei

function I_aligned = align_bands(I, max_shift, window_size, crop_orig)

    switch nargin
        case 4
            I_aligned = align_bands_(I, max_shift, window_size, crop_orig);
        case 3
            I_aligned = align_bands_(I, max_shift, window_size);
        case 2
            I_aligned = align_bands_(I, max_shift);
        case 1
            I_aligned = align_bands_(I);
        otherwise
            error('Error in input arguments');
    end

end