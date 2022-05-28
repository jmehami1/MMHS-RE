% Description
%
%   function [L_PAIR,  MAPPING, opt_lambda, opt_theta, opt_q] = graph_cut_optimise(I, LABELS, KPMASK, SPMATRIX, CNRPOINTS, pitch_height, pitch_width, THETA, LAMBDA, Q)
%   This function is to find the pair of labels in given Labels using
%   specified values of theta, lambda and Q. 
%
%   Input:
%       I:          Input image data cube
%       LABELS:     given label set
%       KPMASK:     the mask of pitch. 
%       SPMATRIX:   the specularity matrix of given image
%       CNRPOINTS:  the coordinates of left bottom corner points of pitches 
%       pitch_height and pitch_width: the dimension of pitches, both
%                   default to 20
%       THETA:      values of theta candidates, default to 0.001
%       LAMBDA:     values of lambda candidates, default to 0.7
%       Q:          values of q candidates, default to 1
%   Output:
%       L_PAIR:     selected pair of labels
%       MAPPING:    the mapping matrix for each pitch to labels
%       opt_lambda: optimal value of lambda
%       opt_q:      optimal value of q
%       opt_theta:  optimal value of theta
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2015 All Rights Reserved.
% Author: Ran Wei

function [L_PAIR, MAPPING, opt_lambda, opt_theta, opt_q] = graph_cut_optimise(I, LABELS, KPMASK, SPMATRIX, CNRPOINTS, pitch_height, pitch_width, THETA, LAMBDA, Q)

    switch nargin
        case 10
            [L_PAIR, MAPPING, opt_lambda, opt_theta, opt_q] = graph_cut_optimise_(I, LABELS, KPMASK, SPMATRIX, CNRPOINTS, pitch_height, pitch_width, THETA, LAMBDA, Q);
        case 9
            [L_PAIR, MAPPING, opt_lambda, opt_theta, opt_q] = graph_cut_optimise_(I, LABELS, KPMASK, SPMATRIX, CNRPOINTS, pitch_height, pitch_width, THETA, LAMBDA);
        case 8
            [L_PAIR, MAPPING, opt_lambda, opt_theta, opt_q] = graph_cut_optimise_(I, LABELS, KPMASK, SPMATRIX, CNRPOINTS, pitch_height, pitch_width, THETA);
        case 7
            [L_PAIR, MAPPING, opt_lambda, opt_theta, opt_q] = graph_cut_optimise_(I, LABELS, KPMASK, SPMATRIX, CNRPOINTS, pitch_height);
        case 6
            [L_PAIR, MAPPING, opt_lambda, opt_theta, opt_q] = graph_cut_optimise_(I, LABELS, KPMASK, SPMATRIX, CNRPOINTS);
        otherwise
            error('error in arguments');
    end
end
