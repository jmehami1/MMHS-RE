%   Description
%       [L, MAPPING, KPMASK, SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM(I, method, theta, pitch_height, pitch_width, order)
%   recovers multiple illuminant from given images.
%
%   Input:  
%       I:            Image data cube
%       method:       a string specifying which method will be used.
%                     Available options are: 'grey_edge', 'grey_world',
%                     'maxrgb' and 'shades_of_grey'. Default value is
%                     'grey_edge'. 
%       theta:        values of theta. It could be a vector or a single
%                     value variable, defaults are [0.001, 0.03, 0.1, 3, 10].
%       pitch_height: height of pitches used in this function, default to 20.
%       pitch_width:  width of pitches used in this function, default to 20
%       order:        differential order default to 1
%   Ouptut:
%       L:            illuminants estimated, in row format
%       MAPPING:      matrix indicates each pitches is assigned to which
%                     illuminant
%       SPMATRIX:     Specularity mask
%       CNRPOINTS:    matrix holds coordinates of top left corners of
%                     pitches
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2015 All Rights Reserved.
% Author: Ran Wei. This code is based on the code of Lawrence Mutimbu of NICTA


function [L, MAPPING,  SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM(I, method, theta, pitch_height, pitch_width, order)
    
    switch nargin
        case 6
            [L, MAPPING, SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM_(I, method, theta, pitch_height, pitch_width, order);
        case 5
            [L, MAPPING, SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM_(I, method, theta, pitch_height, pitch_width);
        case 4
            [L, MAPPING, SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM_(I, method, theta, pitch_height);
        case 3
            [L, MAPPING, SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM_(I, method, theta);
        case 2
            [L, MAPPING, SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM_(I, method);
        otherwise
            [L, MAPPING, SPMATRIX, CNRPOINTS] = recover_multi_illuminant_LM_(I);
    end
            
end