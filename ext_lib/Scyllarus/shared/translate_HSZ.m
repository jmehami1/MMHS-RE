% Re-evaluation routine for HSZ data structures
%
% Syntax:
%     HSZ = translate_HSZ(HS, wavelength);
% 
% Description:
%     Re-evaluates the Scyllarus data struct HS using the vector of wavelengths in WAVELENGTH
% 
% Input:
%     HS:         Input Scyllarus data structure
%     wavelength: Vector of wavelengths to use for the re-evaluation of the HSZ data struct
% 
% Output:
%     HSZ: HSZ data structure. This is RAW encoded. For encodings other than RAW, use the
%           RAW, use the encode_HSZ routine.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function HSZ = translate_HSZ(HSPipeline, WAVELENGTH)
    
    HSZ = translate_HSZ_(HSPipeline, WAVELENGTH);
   
end