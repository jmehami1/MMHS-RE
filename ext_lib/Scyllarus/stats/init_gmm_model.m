% Syntax
%       model = init_gmm_model(I);
%       model = init_gmm_model(I, INDEX);
%       model = init_gmm_model(I, INDEX, ndims);
%       model = init_gmm_model(I, [], ndims);
%       model = init_gmm_model(I, INDEX, ndims, ngmm);
%       model = init_gmm_model(I, [], ndims, ngmm);
%       
% Description:
%   This function is used to initialise the General Mixture Model based on the paper by 
%   Ravindran Kannan et al. entitled "The Spectral Method for General
%   Mixture Models" (COLT 2005).
% 
% Input: 
% 
%   I:     the radiance image (stored as a 3D array with size height x width x bands).
%   INDEX: Indices for the pixels in the image to be sampled for the estimation process. 
%          If INDEX is empty, all pixels are considered to be valid.
%   ndims: Number of dmensions in which the data is to be subspace-projected. It is set, by default, to six.
%   ngmm:  Number of mixtures used for the initialisation. This is only used if no initmodel is provided and, by
%          default, is set to 3; 
% 
% Output: 
%
%   model: Structure containing the initial general mixture model. This contains the following fields:
%          ngmm:       Number of components for the mixture
%          T_idx:      Cell array containing the indeces for the pixels classified based upon the distance to the
%                      mixture means. 
%          centres_w:  Means for the weighted mixture weights
%          covars_w:   Covariance matrices for the weighted mixtures.
%          centres:    Means for the unweighted mixture weights
%          covars:     Covariance matrices for the unweighted mixtures.
%          Uq:         Principal components for the sampled pixels. This is computed using SVD.
%          EM_options: Set of options used for the em_gmm routine
%          prioL:      Prior for each of the components over the data (image pixels).
%          tip_record: Value of the pixel (instance) in the data which corresponds to the simplex that is better aligned
%                      to the mixture. 
%
% See also:
%       train_gmm_model
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Lin Gu, Ran Wei, Antonio Robles-Kelly
    
function model = init_gmm_model(I, INDEX, ndims, ngmm)

    switch nargin
        case 4
            model = init_gmm_model_(I, INDEX, ndims, ngmm);
        case 3
            if ~exist('INDEX', 'var') || isempty(INDEX)
                model = init_gmm_model_(I, [], ndims);
            else
                model = init_gmm_model_(I, INDEX, ndims);
            end            
        case 2
            model = init_gmm_model_(I, INDEX);
        case 1
            model = init_gmm_model_(I);
        otherwise
            error('Incorrect input arguments');
    end

end
