%   Syntax
%       Y = lblvec2mat(y, nc)
%
%   subroutine to convert 
%		1. label vector R_n in {0,1,..,K} to label matrix R_{nxK} (lblvec2mat)
%		2. label image hxw to label matrix image hxwxk
%       this is the inverse of routine lblmat2vec
%       make sure y is an integer array/matrix
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Antonio Robles-Kelly

function Y = lblvec2mat(y, nc)
    
    Y = lblvec2mat_(y, nc);
    
end
