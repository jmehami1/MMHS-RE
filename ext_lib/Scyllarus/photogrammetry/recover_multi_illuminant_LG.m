% Multiple-illuminant recovery routine. Multiple-illuminant recovery routine. Note: this method is experimental
% and is subject to further modification without notice. 
%
% Syntax
%   [L, model] = recover_multi_illuminant(I);
%   [L, model] = recover_multi_illuminant(I, options);
%
% Description:
%   Computes the power spectrum of multiple light sources in a scene.
%
% Inputs:
%   I:       3D hyperspectral image data cube. It contains height X width X bands pixels.
%   options: Structure with the following fields
%            bitdepth: Is the data type for the spectral cube, i.e. number of bits per
%                      spectral measurement. By fault this is 16.
%            method:   Selects between the following methods 
%                      'HRK': Employs the method of Huynh and Robles-Kelly (A Solution of the 
%                             Dichromatic Model for Multispectral Photometric Invariance, International 
%                             Journal of Computer Vision 2010).
%                      'FS': Uses the method of Finlayson and Schaefer (Convex and Non-convex Illuminant 
%                            Constraints for Dichromatic Colour Constancy, CVPR 2001).
%                      'GW': Uses the Grey World method.
%                      'SG': Uses the Shade of Grey method.
%                      'WP': Uses the White Patch method.
%                      '1stOGE': Uses the 1st order Grey Edge method.
%                      '2ndOGE': Uses the 2nd order Grey Edge method.
%            alpha:    The value for the regularisation term used for the HRK
%                      (Huynh and Robles-Kelly) method. The default for this is 50. 
%            patches:  Pre-selected patches. This could be a set of geometry data of patches
%                      with a format of (Top_left_y, top_left_x, height, width). This can be left empty.
%            debug:    Defines the level of debugging information shown at execusion time (debug<3).
%                      the default is 0. 
%
% Outputs:
%   L:     illuminants recovered from given arguments.
%   model: Structure containing the initial general mixture model. This
%          contains the following fields
%          q:          Number of components for the mixture
%          T_idx:      Cell array containing the indeces for the pixels classified based 
%                      upon the distance to the mixture means.
%          centres_w:  Means for the weighted mixture weights
%          covars_w:   Covariance matrices for the weighted mixtures.
%          centres:    Means for the unweighted mixture weights
%          covars:     Covariance matrices for the unweighted mixtures.
%          Uq:         Principal components for the sampled pixels. This is computed using SVD.
%          EM_options: Set of options used for the em_gmm routine
%          prioL:      Prior for each of the components over the data (image pixels).
%          tip_record: Value of the pixel (instance) in the data which corresponds to the simplex 
%                      that is better aligned to the mixture.
%          pLx_map:    Map of labels for the data accroding to the GMM and the
%                      Markov random field used for the train_gmm_model.
%          pLx_prob:   Map of probabilities for the data as delivered by the Markov random field.
%
%   See also:
%
%       recover_global_illuminant, recover_dichromatic_parameters
%
%   Notes:
%
%   This routine requires the use of the randwalk function and, hence, 
%   the SuiteSparse package, which can be downloaded from 
%   http://www.cise.ufl.edu/research/sparse/SuiteSparse/
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Lin Gu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [L, model] = recover_multi_illuminant(I, options)
    
    switch nargin
        case 2
            [L, model] = recover_multi_illuminant_LG_(I, options);
        case 1
            [L, model] = recover_multi_illuminant_LG_(I);
        otherwise
            error('Incorrect input arguments');
    end
    
end