%   This function is designed to train a 3D SOM neuron network using given hyperspectral image cube data
%
% Syntax
%    updates = som3d_training(NEURONS, I, options, loopnum, seed)
%
% Description
%
%   Train a 3D SOM neuron network using given hyperspectral image cube data
%    
% Input:
%
%   NEURONS: the neuron network matrix in 2D form which is of size (bands, length)
%
%   I2D:     2D training data converted from 3D Hyperspectral data cube - size (bands, height by width)
%
%   options: options includes height, width, sigma and learn_rate. height and width are first two dimensions of NEURON size.
%            sigma and learn_rate are options used to update neurons
%
%   loopnum: the number of total training steps. Default value is 100,000.
%
%   seed:  the seed used by randi function. Default is MATLAB default value
%
% Output:
%   
%   updates: the number of updates performed during this call
%   UPR    : map of update number on every neurons in 1D format. length of
%            UPR is the same as neuron number. values in UPR indicates
%            updates happened on that neuron.
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function [TRAINED_N2D, UPR] = som3d_training(N2D, I2D, options, loopnum)

    switch nargin
        case 5
            [TRAINED_N2D, UPR] = som3d_training_(N2D, I2D, options, loopnum, seed);
        case 4
            [TRAINED_N2D, UPR] = som3d_training_(N2D, I2D, options, loopnum);
        case 3
            [TRAINED_N2D, UPR] = som3d_training_(N2D, I2D, options);
        otherwise
            error('Incorrect input argument');
    end
    
end
    