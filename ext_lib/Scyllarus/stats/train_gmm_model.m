% Syntax
%   model = train_gmm_model(I);
%   model = train_gmm_model(I, initmodel);
%   model = train_gmm_model(I, initmodel, options);
%
% Description:
%   This function is used to recover the General Mixture Model (GMM) based on the paper by 
%   Ravindran Kannan et al. entitled "The Spectral Method for General
%   Mixture Models" (COLT 2005).
% 
% Input: 
%   I: the radiance image (stored as a 3D array with size height x width x bands).
%   initmodel: Initial value of the general Mixture Model.
%   options: Structure containing the following fields
%       use_gaussian: If this variable is set ot unity, the GMM model is
%           recovered using a Gaussian prior. If this field is set to zero, a
%           kernel density estimator (KDE) is used instead. The default is
%           options.use_gaussian = 0.
%       ndims: Number of dmensions in which the data is to be subspace-projected.
%           If ndims is not provided, it is automatically set to 6.
%       ngmm: Number of mixtures used for the initialisation. This is only
%           used if no initmodel is provided and, by default, is set to 3;
% 
% Output: 
%   model: Structure containing the initial general mixture model. This contains the following fields
%           q: Number of components for the mixture
%           T_idx: Cell array containing the indeces for the pixels classified based 
%             upon the distance to the mixture means.
%           centres_w: Means for the weighted mixture weights
%           covars_w: Covariance matrices for the weighted mixtures.
%           centres: Means for the unweighted mixture weights
%           covars: Covariance matrices for the unweighted mixtures.
%           Uq: Principal components for the sampled pixels. This is computed using SVD.
%           EM_options: Set of options used for the em_gmm routine
%           prioL: Prior for each of the components over the data (image pixels).
%           tip_record: Value of the pixel (instance) in the data which corresponds to the simplex 
%             that is better aligned to the mixture.
%           pLx_map: Map of labels for the data accroding to the GMM and the
%             Markov random field used for the train_gmm_model.
%           pLx_prob: Map of probabilities for the data as delivered by the
%             Markov random field.
%
%   See also:
%
%       init_gmm_model, em_kde, em_gmm, randwalk
%
%   Notes:
%
%   This routine requires the use of the randwalk function and, hence, 
%   the SuiteSparse package, which can be downloaded from 
%   http://www.cise.ufl.edu/research/sparse/SuiteSparse/
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Lin Gu

function model = train_gmm_model(I, initmodel, options)

    switch nargin
        case 3
            model = train_gmm_model_(I, initmodel, options);
        case 2
            model = train_gmm_model_(I, initmodel);
        case 1
            model = train_gmm_model_(I);
        otherwise
            error('Not enough input arguments');
    end

end