%   Recover materials from given hyperspectral image cube by using Self Organising Map (SOM) of neural network.
%   [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width, som3d_depth, SOM, loopnum, seed)
%   
%   Input
%
%       I           : A 3D hyperspectral image cube
%       som3d_height: Height of som neuron network matrix, Default to 10
%       som3d_width : Width of som neuron network matrix, Default to 10
%       som3d_depth : depth of som neuron network matrix, Default to 10
%       SOM         : Optional pre-set neuron network matrix
%       loopnum     : is the number of loops to be conducted in training,
%                     default to 100,000
%       seed  :  the seed used by randi function. Default is MATLAB default value
%
%   Output
%
%       WINNER: material index categorised. A 2D matrix which has the same
%               frame size as the original image.
%       MSSOM : the SOM neurons after meanshift.
%       COUNT : pixel number for each neurons (material recognised)
%       SOM   : The neuron network used in this computation. It's in 2D
%               form (bands by neuron number). This SOM is not clusterred by
%               meanshift.
%
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei

function [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM(I, som3d_height, som3d_width, som3d_depth, SOM, loopnum, seed)

    switch nargin
        case 7
            [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width, som3d_depth, SOM, loopnum, seed);
        case 6
            [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width, som3d_depth, SOM, loopnum);
        case 5
            [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width, som3d_depth, SOM);
        case 4
            [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width, som3d_depth);
        case 3
            [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width);
        case 2
            [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I, som3d_height);
        case 1
            [WINNER, MSSOM, SOM, COUNT] = recover_materials_SOM_(I);
        otherwise
            error('Incorrect input arguments');
    end
    
end