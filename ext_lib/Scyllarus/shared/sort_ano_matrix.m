% sort_ano_matrix sorts TARGET matrix according to the result of
% sort(SOURCE, dim, MODE).   
%
%           RESULT = sort_ano_matrix(SOURCE, TARGET, dim, MODE)
%
%   Input:
%       SOURCE and TARGET: 2D or 3D matrix which should have identical size. 
%       dim:  the dimension along which the sorting operation goes. By
%             default, dim is given the value of 1 which instructs it to
%             sort the matrix along height direction. MODE selects the
%             direction of the sort  
%       MODE: 
%           'ascend' results in ascending order
%           'descend' results in descending order. 'ascend' is default.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei

function RESULT = sort_ano_matrix(SOURCE, TARGET, dim, MODE)
 
%   by default, sorting is carried on ascending
    if ~exist('MODE', 'var')
        MODE = 'ascend';
    end
    if ~exist('dim', 'var')
        dim = 1;
    end

    if ~isequal(size(SOURCE), size(TARGET))
        error('Dimensions of SOURCE and TARGET do not match');
    end

    [~, ~, b] = size(SOURCE);

    if (b == 1)
        RESULT = sort_ano_matrix2d_(SOURCE, TARGET, dim, MODE);
    else
        RESULT = sort_ano_matrix3d_(SOURCE, TARGET, dim, MODE);
    end

end