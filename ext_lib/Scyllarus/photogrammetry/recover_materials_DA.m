% Syntax
%     [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate, max_clu_num, tmax, tmin, cool_rate, split_threshold) 
%   
%   Clustering a given set of feature vectors using the deterministic
%   annealing approach introduced by Kenneth Rose, Deterministic Annealing
%   for Clustering, Compression, Classification, Regression and Related
%   Optimization Problems, Proceeding IEEE, 1998. 
%    
% Input:
%   S:               3D matrix of source image reflectance in vector format(height by width by bands) 
%   debug:           The level of debugging information to be shown. It
%                    ranges from 1 to 3, 1 (default) for the minimal information
%                    while 5 means the most detailed description. 
%   tmax:            The maximum temperature of the deterministic annealing process. (default: 0.02)
%   tmin:            The minimum temperature of the deterministic annealing process (default 0.00025)
%   cool_rate:       The cooling factor in each iteration of the outer loop. (default: 0.8)
%   max_clu_num:     The maximum number of clusters. (default: 20)
%   split_threshold: The (dot product) threshold below which a cluster should be split. When the dot
%                    product between the real and the surrogate centroid vectors fall below this
%                    threshold, then the cluster needs to be split. (default: cos(5*pi/180))
%   drate:           the rate that the input image is down sampled at. Default is 1 (no down-sampling)
%   
%   The distance metric used in this implementation is the spectral angle
%   (or the dot product) between any two normalised reflectance spectra.
%
% Output:
%   MAT:    materials found by this function. (materials x bands) 
%   MATAB:  The material abundancy matrix for each pixel (height x width x materials number) 
%   MATIND: The material index image for each pixel (height x width)
%   
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function [MAT, MATIND, MATAB] = recover_materials_DA(S, debug, drate, max_clu_num, tmax, tmin, cool_rate, split_threshold) 
    
    switch nargin
        case 8
            [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate, max_clu_num, tmax, tmin, cool_rate, split_threshold);
        case 7
            [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate, max_clu_num, tmax, tmin, cool_rate);
        case 6
            [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate, max_clu_num, tmax, tmin);
        case 5
            [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate, max_clu_num, tmax);
        case 4
            [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate, max_clu_num);
        case 3
            [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate);
        case 2
            [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug);
        case 1
            [MAT, MATIND, MATAB] = recover_materials_DA_(S);
        otherwise
            error('Incorrect input arguments');
    end
    
end