% function [PATCHES, ...
%           LIGTH_EST_MASK, ...
%           CONTRAST_MASK, ...
%           HIGHLIGHT_MASK, ...
%           PATCH_MAP] = select_smooth_patches(I, ...
%                                              pheight, ...
%                                              pwidth, ...
%                                              mean_thresh, ...
%                                              selected_num, ...
%                                              debug)                                                 
%
% Select smooth patches with the most contrast (averaged over all bands) due
% to specularity (not because the patch spans different materials).
%
% Input:
%
%   I:              height x width x bands - the dimensions of input data
%   pheight:        height of each patch, default to 20.
%   pwidth:         width of each patch, default to 20.
%   selected_num:   number of selected patches which is default to 50
%   mean_thresh:    only choose patches with mean radiance across pixels and bands greater than this 
%                   threshold. Default to 100
%   debug:          debug information will shown; values range from 0 to 2
%
% Output:
%
%   LIGTH_EST_MASK: a mask showing 1 at the pixels selected for initial light
%                   estimation, 0 otherwise.
%   CONTRAST_MASK:  1 where the most contrast patches are selected, 0 otherwise (same size as
%                   image)
%   HIGHLIGHT_MASK: the mask corresponding to patches where specularities
%                   need to be removed.
%   PATCH_MAP:      if a patch with coordinates i,j is selected, then patchMap(i, j) = 1.
%   PATCHES:        selected patches.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014-2015 All Rights Reserved.
% Author: Ran Wei

function [PATCHES, ...
          LIGTH_EST_MASK, ...
          CONTRAST_MASK, ...
          HIGHLIGHT_MASK, ...
          PATCHE_MAP] = select_smooth_patches(I, ...
                                              p_height, ...
                                              p_width, ...
                                              mean_thresh, ...
                                              selected_num, ...
                                              debug)
                                             
    switch nargin
        case 6
            [PATCHES, ...
             LIGTH_EST_MASK, ...
             CONTRAST_MASK, ...
             HIGHLIGHT_MASK, ...
             PATCHE_MAP] = select_smooth_patches_(I, ...
                                                  p_height, ...
                                                  p_width, ...
                                                  mean_thresh, ...
                                                  selected_num, ...
                                                  debug);
        case 5
            [PATCHES, ...
             LIGTH_EST_MASK, ...
             CONTRAST_MASK, ...
             HIGHLIGHT_MASK, ...
             PATCHE_MAP] = select_smooth_patches_(I, ...
                                                  p_height, ...
                                                  p_width, ...
                                                  mean_thresh, ...
                                                  selected_num);
        case 4
            [PATCHES, ...
             LIGTH_EST_MASK, ...
             CONTRAST_MASK, ...
             HIGHLIGHT_MASK, ...
             PATCHE_MAP] = select_smooth_patches_(I, ...
                                                  p_height, ...
                                                  p_width, ...
                                                  mean_thresh);
                                              
        case 3
            [PATCHES, ...
             LIGTH_EST_MASK, ...
             CONTRAST_MASK, ...
             HIGHLIGHT_MASK, ...
             PATCHE_MAP] = select_smooth_patches_(I, ...
                                                  p_height, ...
                                                  p_width);
        case 2
            [PATCHES, ...
             LIGTH_EST_MASK, ...
             CONTRAST_MASK, ...
             HIGHLIGHT_MASK, ...
             PATCHE_MAP] = select_smooth_patches_(I, ...
                                                  p_height);
        case 1
            [PATCHES, ...
             LIGTH_EST_MASK, ...
             CONTRAST_MASK, ...
             HIGHLIGHT_MASK, ...
             PATCHE_MAP] = select_smooth_patches_(I);
        otherwise
            error('Not enough input arguments');
    end

end