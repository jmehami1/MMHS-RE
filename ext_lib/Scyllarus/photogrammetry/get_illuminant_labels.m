%   Description:
%   [LABELS, KPMASK, SPECULARITY, CORNORS] = get_illuminant_labels(I, pitch_height, pitch_width, method, order)
%   
%   This function is used to produce illuminant labels from pitches using
%   input image data cube and selected methods.
%
%   Input:
%       I:            image data cube
%       pitch_height: height for pitches, default to 20.
%       pitch_width:  width for pitches, default to 20.
%       method:       estimation method: Available options are: 'grey_edge', 'grey_world',
%                     'maxrgb' and 'shades_of_grey'. Default value is 'grey_edge'. 
%       order:        differential order. default to 1
%
%   Output:
%       LABELS:       illuminant estimation labels
%       SPECULARITY:  specularity matrix
%       CORNORS:      coordinates of top left corners of pitch 
%   
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2015 All Rights Reserved.
% Author: Ran Wei. This code is based on the code of Lawrence Mutimbu of NICTA


function [LABELS, KPMASK, SPECULARITY, CORNORS] = get_illuminant_labels(I, pitch_height, pitch_width, method, order)
    
    switch nargin
        case 5
            [LABELS, KPMASK, SPECULARITY, CORNORS] = get_illuminant_labels_(I, pitch_height, pitch_width, method, order);
        case 4
            [LABELS, KPMASK, SPECULARITY, CORNORS] = get_illuminant_labels_(I, pitch_height, pitch_width, method);
        case 3
            [LABELS, KPMASK, SPECULARITY, CORNORS] = get_illuminant_labels_(I, pitch_height, pitch_width);
        case 2
            [LABELS, KPMASK, SPECULARITY, CORNORS] = get_illuminant_labels_(I, pitch_height);
        case 1
            [LABELS, KPMASK, SPECULARITY, CORNORS] = get_illuminant_labels_(I);
        otherwise
            error('Incorrect input arguments');
    end
    
end