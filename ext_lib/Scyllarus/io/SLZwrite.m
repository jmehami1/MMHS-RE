% SLZwrite library data into a SLZ file. 
%
% Syntax
%
%     SLZwrite(filename, DATA, options);
% 
% Description
%
%   Write a library data structure (SLZ) to disk in HDF5 format.
%
% Inputs:
%     filename: The name of the file (including the path) to be written to disk
%     DATA:     The data structure delivered by the HSPipeline routine
%     options:  Write to disk options these are
%               'compression': Level of compression for the HDF5 data (0-9). 
%               The default is 9 (maximum compression).
%               'datatype': Type for the data written to disk on the HDF5 datasets. 
%               The default is 'uint16', but 'uint8' can also be used. 
%             
% Example:
%
%   Write an SLZ file to disk using a medium level of compression and
%   unsigned integers as data type.
%
%      options.compression = 5;
%      options.datatype = 'uint8';
%      SLZwrite('test.slz', DATA, options);
%
% See also:
%      HSZread, HSZwrite, SLZread, FLAread, FLAwrite
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2014 All Rights Reserved.
% Author: Ran Wei & Antonio Robles-Kelly. 

function SLZwrite(filename, DATA, options)

    switch nargin
        case 3
            export_slz_(filename, DATA, options);
        case 2
            export_slz_(filename, DATA);
        otherwise
            error('Incorrect input arguments');
    end
    
end
