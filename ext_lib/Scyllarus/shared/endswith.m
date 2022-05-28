%   this function is designed to test whether a text string is ended by a
%   specified sub-string.
%
%   Description:
%       value = endswith(str, suffix, casesensitive)
%  
%   Input: 
%   
%       str: the string to be tested
%       suffix: substr this function is looking for
%       casesensitive: case sensitive or not. default to 0 (no)
%
%   Output: 
%       value: true when str is ended by suffix or false, otherwise
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function value = endswith(str, suffix, casesensitive)

    switch nargin
        case 3
            value = endswith_(str, suffix, casesensitive);
        case 2
            value = endswith_(str, suffix);
        otherwise
            error('Incorrect input arguments');
    end

end