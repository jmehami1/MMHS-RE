% Evaluation routine for the HSZ data structure
%
% Syntax
%   HSZ = eval_HSZ(HSPipeline);
%   HSZ = eval_HSZ(HSPipeline, wavelength);
%
% Description:
%   Evaluates a HSZ data structure so as to return another struct encoded
%   in RAW measurements over the wavelengths on WAVELENGTH. Note that this
%   routine has no effect if the HSZ structure has been RAW encoded.
%
% Input:
%   wavelength: Vector of target wavelengths. These have to be within the scope of
%               HSPipeline.HDR.wavelength. If this parameter is not provided,
%               HSPipeline.HDR.wavelength is used instead. 
%   HSPipeline: Data structure to be evaluated
%
% Output:
%   HSZ: Data structure encoded in RAW format over the wavelength vector 
%        wavelength.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2014, All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function HSZ = eval_HSZ(HSPipeline, WAVELENGTH)

     if ~exist('WAVELENGTH', 'var')
         HSZ = eval_HSZ_(HSPipeline);
     else
         HSZ = eval_HSZ_(HSPipeline, WAVELENGTH);
     end
     
 end