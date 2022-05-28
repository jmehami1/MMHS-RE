% Read SLZ library files from disk
%
% Syntax:
%     SLZ = SLZread(filename);
%
% Description
%     SLZread reads spectral library files (SLZ) from disk.
%
% Input:
%      filename: The name of the file (including the path and extension)
%            to be read from disk.
%
% Output:
%      SLZ: Data structure containing the library data. 
%               
% Example:
%      SLZ = SLZread('test.slz');
%
% See dalso:
%      HSZread, HSZwrite, SLZwrite, FLAread, FLAwrite
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly. 

function SLZ = SLZread(filename)

    SLZ = import_slz_(filename);

end
