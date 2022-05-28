% Recover the set of materials in the scene
%
% Syntax
%   [MAT, MATIND, MATAB] = recover_materials(S);
%   [MAT, MATIND, MATAB] = recover_materials(S, [], debug); 
%   [MAT, MATIND, MATAB] = recover_materials(S, [],[], drate);
%   [MAT, MATIND, MATAB] = recover_materials(S, [],[],[], max_clu_num) 
%   [MAT, MATIND, MATAB] = recover_materials(S, [],[],[], [], tmax):
%   [MAT, MATIND, MATAB] = recover_materials(S, [],[],[], [], [], tmin):
%   [MAT, MATIND, MATAB] = recover_materials(S, [],[],[], [], [], [],cool_rate):
%   [MAT, MATIND, MATAB] = recover_materials(S, [],[],[], [], [], [], [], split_threshold); 
%   [MAT, MATIND, MATAB] = recover_materials(S, method, debug,...
%               drate, max_clu_num, tmax, tmin, cool_rate, split_threshold);
%
% Description
%
%   Clustering a given set of feature vectors using either
%   the deterministic annealing approach introduced by Kenneth Rose (Deterministic 
%   Annealing for Clustering, Compression, Classification, Regression and Related
%   Optimization Problems, Proceeding IEEE, 1998) or K-means.
%   The distance metric used in this implementation is the spectral angle
%   (or the dot product) between any two normalised reflectance spectra.
%    
% Input:
%   S:               3D matrix of source image reflectance in vector format (height by width by bands) 
%   method:          String denoting the method to be used, i.e. 'DA' or
%                    'KM' for either deterministic annealing or k-means,
%                    respectively. The default is determinsitic annealing ('DA').
%   debug:           The level of debugging information to be shown. It
%                    ranges from 1 to 5, 1 (default) for the minimal information
%                    while 5 means the most detailed description. 
%   tmax:            The maximum temperature of the deterministic annealing process. (default: 0.02)
%   tmin:            The minimum temperature of the deterministic annealing process (default 0.00025)
%   cool_rate:        The cooling factor in each iteration of the outer loop. (default: 0.8)
%   max_clu_num:     The maximum number of clusters. (default: 20)
%   split_threshold: The (dot product) threshold below which a cluster should be split. When the dot
%                    product between the real and the surrogate centroid vectors fall below this
%                    threshold, then the cluster needs to be split. (default: cos(5*pi/180))
%   drate:           the rate that the input image is down sampled at. Default is 1 (no downsampling)
%                    (default: 1)
%   
%
% Output:
%   MAT:    materials found by this function. (materials x bands) 
%   MATAB:  The material abundancy matrix for each pixel (height x width x materials number) 
%   MATIND: The material index image for each pixel (height x width)
%   
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function [MAT, MATIND, MATAB] = recover_materials(S, method, debug, drate, max_clu_num, tmax, tmin, cool_rate, split_threshold) 

    [~, ~, bands] = size(S);
    
    %   handle and verify input parameters.
    if ~exist('split_threshold', 'var') || numel(split_threshold) ~= 1 || split_threshold(1) < 0 ...
       || isempty(split_threshold)
       split_threshold = cos(5*pi/180);
    end
    if ~exist('cool_rate','var') || numel(cool_rate) ~= 1 || cool_rate(1) < 0  ...
       || isempty(cool_rate)
       cool_rate = 0.8;
    end
    
    if ~exist('tmin','var') || numel(tmin) ~= 1 || tmin(1) < 0  ...
       || isempty(tmin)
       tmin = 0.00025;
    end
    
    if ~exist('tmax','var') || numel(tmax) ~= 1 || tmax(1) < 0  ...
       || isempty(tmax)
       tmax = 0.02;
    end
    
    
    if ~exist('method', 'var') || numel(debug) ~= 1 || strcmp(method,'KM')
       method = 'DA';
    end
    
    if ~exist('drate', 'var') || numel(drate) ~= 1 || drate(1) < 0 ...
       || isempty(drate)
       if strcmpi(method,'DA')
           drate = 1;
       else
           drate = 5;
       end
    end
    
    if ~exist('max_clu_num','var') || numel(max_clu_num) ~= 1 || max_clu_num(1) < 0 || isempty(max_clu_num)
        max_clu_num = 20;
    end
    
    if ~exist('debug', 'var') || numel(debug) ~= 1 || debug(1) < 0
       debug = 1;
    end
    
    if strcmpi(method,'DA')
        [MAT, MATIND, MATAB] = recover_materials_DA_(S, debug, drate, max_clu_num, tmax, tmin, cool_rate, split_threshold);
    elseif strcmpi(method,'KM')
        [MAT, MATIND, MATAB] = recover_materials_KM_(S, max_clu_num, debug, drate);
    else
        error('Unknown recovering method');
    end
    %   Normalise the material matrix and finish off
    norm = sqrt(sum(MAT .^2, 2));
    norm(norm == 0) = 1;
    MAT = MAT./ norm(:, ones(bands, 1));
    
end