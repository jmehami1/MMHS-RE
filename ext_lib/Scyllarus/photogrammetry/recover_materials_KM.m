%   function [MAT, MATIND, MATAB] = recover_materials_KM(S, max_clu_num, debug, drate) 
%   
%   Clustering a given set of feature vectors using k-means. 
%    
% Input:
%
%   S:               3D matrix of source image reflectance in vector format (height by width by bands) 
%   debug:           The level of debugging information to be shown. It
%                    ranges from 1 to 5, 1 (default) for the minimal information
%                    while 5 means the most detailed description. 
%   max_clu_num:     The maximum number of clusters. (default: 20)
%   drate:           the rate that the input image is down sampled at. (Default is 1 - no downsampling) 
%   
% Output:
%
%   MAT:    materials found by this function. (materials x bands) 
%   MATAB:  The material abundancy matrix for each pixel (height x width x materials number) 
%   MATIND: The material index image for each pixel (height x width)
%   
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function [MAT, MATIND, MATAB] = recover_materials_KM(S, max_clu_num, debug, drate) 

    switch nargin
        case 4
            [MAT, MATIND, MATAB] = recover_materials_KM_(S, max_clu_num, debug, drate);
        case 3
            [MAT, MATIND, MATAB] = recover_materials_KM_(S, max_clu_num, debug);
        case 2
            [MAT, MATIND, MATAB] = recover_materials_KM_(S, max_clu_num);
        case 1
            [MAT, MATIND, MATAB] = recover_materials_KM_(S);
        otherwise
            error('Incorrect input arguments');
    end
    
end
    