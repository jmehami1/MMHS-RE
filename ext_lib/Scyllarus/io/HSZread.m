% Load data from an HSZ file.
%
% Syntax:
%     HSZ = HSZread(FILENAME);
%     HSZ = HSZread(FILENAME, scale);
%     HSZ = HSZread(FILENAME, rows, cols);
%     HSZ = HSZread(FILENAME, rect);
%
% Description
%   HSZread imports data from a compressed HSZ file. This is an HDF5 file 
%   with all the variables recovered by the Scyllarus routine. 
%
%
% Input:
%     FILENAME: The name of the file (including the path and extension) to 
%           be read from disk.
%     rows, cols: Image cube dimensions. This effectively resizes the
%           hyperspectral image
%     scale: Scale up to which the image is to be resized at loading time.
%     rect: Used to crop the image at loading time. rect is a four-element 
%           position vector[xmin ymin width height] that specifies the size 
%           and position of the crop rectangle. 
%
% Output:
%     HSZ: HSZ data Structure. This is the same as the output of the 
%           Scyllarus routine. 
%
% See also:
%
%   FLAwrite, FLAread, HSZwrite, SLZwrite, SLZread
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function HSZ = HSZread(filename, x, y)

    HSZ = import_hsz_(filename);
    
    %Do the scaling and resizing if it applies
    if exist('x', 'var')
        if ~exist('y', 'var')
            if length(x) == 1
                HSZ = resize_image_(HSZ, x);
            else
                HSZ = crop_image_(HSZ, x);
            end
        else
            HSZ = resize_image_(HSZ, x, y);
        end
    end
    
    %Check that the NURBS degree is included if not present. Note this is
    %not necessary and the default is 2
    if ~isfield(HSZ.HDR, 'degreeNURBSL') && strcmpi(HSZ.HDR.EncodingL,'NURBS')
                HSZ.HDR.degreeNURBSL = 2;
    end
    if ~isfield(HSZ.HDR, 'degreeNURBSS') && strcmpi(HSZ.HDR.EncodingS,'NURBS')
                HSZ.HDR.degreeNURBSS = 2;
    end
    if ~isfield(HSZ.HDR, 'degreeNURBSK') && strcmpi(HSZ.HDR.EncodingK,'NURBS')
                HSZ.HDR.degreeNURBSK = 2;
    end
    
end