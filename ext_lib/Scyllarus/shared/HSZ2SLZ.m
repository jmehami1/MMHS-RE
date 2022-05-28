% Create a library from an indexed HSZ data structure
%
% Syntax
%
%   SLZ = HSZ2SLZ(HSZ, MaterialList, field);
%   SLZ = HSZ2SLZ(HSZ, [], field);
%
% Input
%
%   HSZ:          Scyllarus data structure.
%   MaterialList: Cell array containing the names of the materials or lights to be
%                 stored on the library (SLZ)
%   field:        Determines whether the materials or lights are to be stored. If
%                 the scene materials are to be extracted, field = 'S', otherwise field = 'L'.
%
% Output
%
%   SLZ: Scyllarus library data structure
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function SLZ = HSZ2SLZ(HSZ, MaterialList, field)

    if nargin < 3
        error('Not enough input arguments');
    end
    
    SLZ = HSZ2SLZ_(HSZ, MaterialList, field);
    
end

        
    


