%   this is a function used to do mean shift clustering on a given set of
%   data. For example, a set of neurons for neural network applications.
%   The task is to cluster these data into a set of clusters.
%
%   Description:
%       [MS_MODES, MAPPING] = meanshift(MODES, COUNT)
%  
%   Input: 
%   
%       MODES: mode set to be clusterred. Each element is a vector while the
%          number of elements is determined by the column number (size:
%          depth X mode_number
%       COUNT: the number of pixels that each mode represents
%
%   Output: 
%       MS_MODES:  the final mode matrix after meanshift clustering (size:
%       depth X final_mode_num
%       MAPPING: the mapping between input mode matrix MODES and output MS_MODES
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function [MS_MODES, MAPPING] = meanshift(MODES, COUNT)

    [MS_MODES, MAPPING] = meanshift_(MODES, COUNT);

end