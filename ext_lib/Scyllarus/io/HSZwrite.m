% Write an HSZ data sctructure to disk.
%
% Syntax:
%     HSZwrite(filename, HSZ);
%     HSZwrite(filename, HSZ, options);
% 
% Description
%   HSZwrite exports data in a struct into a compressed NICTA HSPipeline file named by
%   filename which is an HDF5 file with all the variables recovered by the HSPipeline routine.
%
%
% Inputs:
%     filename: The name of the file (including the path) to be written to disk
%     HSZ:      The data structure delivered by the HSPipeline routine
%     options:  Write to disk options these are
%         'compression': Level of compression for the HDF5 data (0-9). 
%             The default is 9 (maximum compression).
%         'datatype': Type for the data written to disk on the HDF5 datasets. 
%             The default is 'uint16', but 'uint8' can also be used 
%             
% See also:
%
%   FLAwrite, FLAread, HSZread, SLZwrite, SLZread
%
% Example:
%
%   Write an HSZ file to disk using a medium level of compression and
%   unsigned integers as data type.
%
%      options.compression = 5;
%      options.datatype = 'uint8';
%      HSZwrite('test.hsz', HSPipeline);
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function HSZwrite(filename, HSZ, options)

    switch nargin 
        case 3
            export_hsz_(filename, HSZ, options);
        case 2
            export_hsz_(filename, HSZ);
        otherwise
            error('Incorrect input arguments');
    end

end