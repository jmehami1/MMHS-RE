% Encoding routine for the HSZ data structure
%
% Syntax:
%     HSZ = encode_HSZ(HSPipeline)
%     HSZ = encode_HSZ(HSPipeline, options)
%
% Description:
%     Encodes an HSZ structure so as to convert it from RAW to a Gaussian mixture model 
%     (GMM) or a spline (NURBS).
%
% Input:
%     HSPipeline: Input RAW encoded HSZ structure
%     options: Structure with the following fields
%         IlluminantEncoding: Determines the manner in which the spectra on 
%             HSZ.L.Elements and HSZ.L.Endmembers is encoded. The default
%             is 'NURBS'. For the Gaussian mixture, use 'GMM'.
%         MaterialEncoding: Determines the manner in which the spectra on 
%             HSZ.S.Elements and HSZ.S.Endmembers 
%             is encoded. The default is 'NURBS'
%         SpecularityEncoding: Encoding scheme used for the highlights at 
%             output. The default is 'NURBS'
%         numGMMsL: Number of mixtures used for the GMM encoding of the
%              illuminant
%         numGMMsS: Number of mixtures used for the GMM encoding of the
%              reflectance
%         numGMMsK: Number of mixtures used for the GMM encoding of the
%              specularity
%
% Output:
%   HSZ: An HSZ structure encoded according to the options above.
% 
% See also
%       eval_HSZ, get_nurbs, get_gaussian_mixture
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2014, All Rights Reserved.
% Author: Antonio Robles-Kelly. 

function HSZ = encode_HSZ(HSPipeline, options)
    
    if ~exist('options', 'var')
        HSZ = encode_HSZ_(HSPipeline);
    else
        HSZ = encode_HSZ_(HSPipeline, options);
    end
    
end