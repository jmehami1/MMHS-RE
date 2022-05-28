%  Syntax
%       model = em_kde(I);
%       model = em_kde(I, init_model);
%       model = em_kde(I, init_model, INDEX);
%       
%   Description:
%       This function is used to compute the General Mixture Model based on the paper by 
%       Ravindran Kannan et al. entitled "The Spectral Method for General
%       Mixture Models" (COLT 2005) using the EM algorithm. It employs at input the model delivered by
%       the routine init_gmm_model and a prior compted via the application of kernel
%       density estimation (KDE).
% 
%   Input: 
%       I:          The radiance image (stored as a 3D array with size height x width x bands).
%       INDEX:      Indices for the valid pixels in the image. If Indx is not provided, all pixels are considered to be valid.
%       init_model: Initial model used for the General Mixture. If this is not provided, the method will employ
%                   init_gmm_model to recover one. 
% 
%   Output: 
%       model: Structure containing the final general mixture model. It has the following members.
%              q:          Number of components for the mixture
%              T_idx:      Cell array containing the indeces for the pixels classified based upon the distance to the
%                          mixture means. 
%              centres_w:  Means for the weighted mixture weights
%              covars_w:   Covariance matrices for the weighted mixtures.
%              centres:    Means for the unweighted mixture weights
%              covars:     Covariance matrices for the unweighted mixtures.
%              Uq:         Principal components for the sampled pixels. This is computed using SVD.
%              EM_options: Set of options used for the em_gmm routine
%              prioL:      Prior for each of the components over the data (image pixels).
%              tip_record: Value of the pixel (instance) in the data which corresponds to the simplex that is better
%                          aligned to the mixture. 
%              pLx_map:    Map of labels for the data accroding to the GMM and the Markov random field used for the
%                          train_gmm_model. 
%              pLx_prob:   Map of probabilities for the data as delivered by the Markov random field.
%
%   See also:
%       init_gmm_model, trian_gmm_model, em_gmm, randwalk
%
%   Notes:
%
%   This routine requires the use of the randwalk function and, hence, 
%   the SuiteSparse package, which can be downloaded from 
%   http://www.cise.ufl.edu/research/sparse/SuiteSparse/
%
%%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei, Lin Gu and Antonio Robles-Kelly

function model = em_kde(I, init_model, INDEX)

    switch nargin
      case 3
        model = em_kde_(I, init_model, INDEX);
      case 2
        model = em_kde_(I, init_model);
      case 1
        model = em_kde_(I);
      otherwise
        error('Incorrect argument number');
    end
    
end