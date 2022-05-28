% Re-sampling routine for spectral data
%
% Syntax:
%       [Q NEW_WAVE] = translate_spectra(S, SOURCE_WAVE, TARGET_WAVE);
% 
% Description:
%       Routine for evaluating the spectra in a SOURCE_WAVE domain different to that
%       in which it was originally sampled. This is done using NURBS
% 
% Input:
%       S:           Matrix of spectra, where the last dimension corresponds to the SOURCE_WAVE domain.
%       SOURCE_WAVE: Vector containing the wavelengths in which the spectra was originally sampled.
%       TARGET_WAVE: Vector containing the wavelengths in which the spectra is to be re-sampled.
%     
% Output:
%       Q: Matrix of re-sampled spectra
%       final_wavelength: Vector of wavelengths used to re-sample Q
% 
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Cong Phuoc Huynh

function [Q, NEW_WAVE] = translate_spectra(S, SOURCE_WAVE, TARGET_WAVE)
    
    [Q, NEW_WAVE] = translate_spectra_(S, SOURCE_WAVE, TARGET_WAVE);
    
end
