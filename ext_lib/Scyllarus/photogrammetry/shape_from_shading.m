% function [SHADE, DATA, WEIGHT] = shape_from_shading_(I, MASK, debug)
%
% Syntax:
%    [SHADE, DATA, WEIGHT] = shape_from_shading(I, MASK, debug) or
%    [SHADE, DATA, WEIGHT] = shape_from_shading(I)
% 
% Description:
%    Recover shapes of objects using only shading information. The algorithm used is half blind signal separation. 
% 
% Input:
%    I:     Hyperspectral data cube of size [height, width, bands].
%    MASK:  the foreground matrix (optional) where 0 for background and 1 for foreground. 
%           Alternatively, if a single numeric value v is given, it will be used as the threshold 
%           of calculating foreground. In that case, MASK = mean(I, 3) > v;
%    debug: Level of debugging information. Ranges from 1 to 3 while 1 is minimum
%    
% Output:
%    SHADE:  result of shape estimated.
%    DATA:   data structure, it also contains MASK used in this function
%    WEIGHT: The MASK used (or generated) in this function. If MASK is given as an input, its output will be the same;
% 
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013-2014 All Rights Reserved.
% Author: Ran Wei and Lin Gu

function [SHAPE, DATA, WEIGHT] = shape_from_shading(I, MASK, debug)

    switch nargin
        case 3
            [SHAPE, DATA, WEIGHT] = shape_from_shading_(I, MASK, debug);
        case 2
            [SHAPE, DATA, WEIGHT] = shape_from_shading_(I, MASK);
        case 1
            [SHAPE, DATA, WEIGHT] = shape_from_shading_(I);
        otherwise
            error('Incorrect number of inputs. Please check');
    end

end
