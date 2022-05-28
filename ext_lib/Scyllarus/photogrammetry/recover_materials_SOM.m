%% %%   Recover materials from given hyperspectral image cube by using Self Organising Map (SOM) of neural network.
%   [WINNER, COUNT, SOM] = recover_materials_SOM_(I, som3d_height, som3d_width, som3d_depth, SOM, loopnum)
%   
%%   Input
%
%       I           : A 3D hyperspectral image cube
%       som3d_height: Height of som neuron network matrix, Default to 10
%       som3d_width : Width of som neuron network matrix, Default to 10
%       som3d_depth : depth of som neuron network matrix, Default to 10
%       loopnum     : is the number of loops to be conducted in training,
%                     default to 100,`000
%
%%   Output
%
%       WINNER: material index categorised. A 2D matrix which has the same
%               frame size as the original image.
%       COUNT : pixel number for each material
%       SOM   : The neuron network used in this computation. It's in 2D
%               form (bands by neuron number)
%
%%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei
% Version: 1.0.2
% Last Update: 22 Sep 2014
%%
function [WINNER, COUNT] = recover_materials_SOM(I, som3d_height, som3d_width, som3d_depth)

    switch nargin
        case 4
            [WINNER, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width, som3d_depth);
        case 3
            [WINNER, COUNT] = recover_materials_SOM_(I, som3d_height, som3d_width);
        case 2
            [WINNER, COUNT] = recover_materials_SOM_(I, som3d_height);
        case 1
            [WINNER, COUNT] = recover_materials_SOM_(I);
        otherwise
            error('Incorrect input arguments');
    end
    
end