%   Description:
%       this is a function used removed unwanted bands from given
%       hyperspectral image data cube. This function will estimate the
%       illuminant power spectrum using given data and remove all bands
%       that have a lower or equal illuminant value than the given threshold. 
%
%   [ND, Ind] = band_remove(D, threshold, bottom)
%  
%   Input: 
%   
%       D:         Hyperspectral image data structure or cube. If a HDR
%                  structure is contained in D, this HDR structure will
%                  also be modified accordingly.
%       threshold: the value of illuminant below which bands will be
%                  removed. 0 is for a band being removed and 1 is for a
%                  band kept.
%       Bottom:    indicates if this function is going to remove dark bands
%                  (<= threshold) or saturated bands (> threshold). By
%                  default it's to remove dark bands.
%
%   Output: 
%   
%       NI:        image data or structure after bands removed
%       Ind:       the indexes of bands removed
%
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function [NI, Ind] = band_remove(I, threshold, bottom)

    switch nargin
        case 3
            [NI, Ind] = band_remove_(I, threshold, bottom);
        case 2
            [NI, Ind] = band_remove_(I, threshold);
        otherwise
            error('Incorrect input arguments');
    end
    
end