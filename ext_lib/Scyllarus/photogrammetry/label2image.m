% Description 
%   function I = label2image(S, CNRPOINTS, MAPPING, LABELS, ph, pw)
%   
% This function is to assign label to pitches of an image using recovered
% illuminants
%   
% Input: 
%   S:          the size information of original image, a vector contains
%               height, width and bands dimensions
%   CNRPOINTS:  coordinates of top left corners of pitches for labelling
%   MAPPING:    mapping matrix indicates which label a pitch belongs to
%   LABELS:     illuminants labels.
%   ph, pw:     height and width of pitches, by default they are 20.
%
% Output:
%   I:          image filled with label illuminants, which is of size S
%
% This computer code is subject to copyright: (c) National ICT Australia
% Limited (NICTA) 2015 All Rights Reserved. 
% Author: Ran Wei


function I = label2image(S, CNRPOINTS, MAPPING, LABELS, ph, pw)
    
    switch nargin
        case 6
            I = label2image_(S, CNRPOINTS, MAPPING, LABELS, ph, pw);
        case 5
            I = label2image_(S, CNRPOINTS, MAPPING, LABELS, ph);
        case 4
            I = label2image_(S, CNRPOINTS, MAPPING, LABELS);
        otherwise
            error('Incorrect input parameters');
    end
    
end



