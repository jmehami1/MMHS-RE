% Compute a trichromatic image from an HSZ or flat file using camera spectral sensitivity functions.
%
% Syntax
%  Irgb = recover_RGBImage(Data);
%  Irgb = recover_RGBImage(Data, CMF);
%  Irgb = recover_RGBImage(Data, CMF, 'use_endmembers');
%  Irgb = recover_RGBImage(Data, CMF, 'tonemap');
%  Irgb = recover_RGBImage(Data, CMF, 'use_endmembers', 'tonemap');
%
% Description:
%     Function for computing a pseudocolour image from an HSZ or FLA image
%     structure using a pre-defined camera spectral sensitivity function.
%
% Inputs:
%
%     Data:             HSZ data struct or FLA image to be processed.
%     CMF:              Colour matching function (CMF) to be used. This is a 4xbands array whose first row contains
%                       the wavelength vector. The remaining rows account for the BGR components of the matching
%                       function. If no CMF is provided, the matching function for the Nikon D70 camera is used. 
%     'use_endmembers': Option which determines whether the endmembers embedded in the file should be used for the color
%                       rendering rather than the materials. By default, the materials are used.
%     'tonemap':        Option which determines whether the image should be tonemapped for display purposes. The default
%                       output is without tonemapping
%
%
% Outputs:
%     Irgb: Pseudo-colour image.
%
% Example:
%
%   I = FLAread('.\shared\samples\face.fla', 'fullheader');
%   HSZ = Scyllarus(I);
%   LibCMF = SLZread('StilesBurch1955CMFs2deg.slz');
%   CMF = LibCMF.Endmember(1:3,:);
%   Irgb = recover_RGBImage(HSZ, [LibCMF.HDR.wavelength; CMF(3,:); CMF(2,:); CMF(1,:)]);
%
%     
% See also:
%
%   FLAread, HSZread, Scyllarus
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function IMAGE = recover_RGBImage(DATA, CMF, enable_endmember, use_tonemap)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Setup the variables
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    use_tonemap = 0;
    
    if ~exist('CMF', 'var') 
        fprintf('Colour matching function not provided. Using spectral response for the Nikon D70 instead.\n');
        load('NikonD70.mat');
        CMF = NikonD70';
    end
    
    if ~exist('enable_endmember', 'var')
        enable_endmember = 0;
        use_tonemap = 0;
    else
        enable_endmember = strcmp(enable_endmember, 'use_endmembers');
        use_tonemap = strcmp(enable_endmember, 'tonemap');
    end
    
     if exist('use_tonemap', 'var')
        use_tonemap = strcmp(use_tonemap, 'tonemap');
     end
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Compute the RGB image
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    IMAGE = recover_RGBImage_(DATA, CMF, enable_endmember);
    
    IMAGE = IMAGE/max(IMAGE(:));
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Do the tonemaping if necessary
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if use_tonemap == 1
        IMAGE = tonemap(IMAGE);
    end
    
end

