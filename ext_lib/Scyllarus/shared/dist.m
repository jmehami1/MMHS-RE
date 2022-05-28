%   Euclidean distance weight function
%  
%   Syntax
%
%       Z = dist(W, P); or If you want find the distance from each neuron
%                       to the other with, use
%       Z = dist(W);
% 
%   Description
% 
%       This function computes weights to an input to get weighted inputs.
% 
%   Inputs: 
%       W:	S-by-R weight matrix
%       P:  R-by-Q matrix of Q input (column) vectors
%   Output:
%       Z:  S-by-Q weight matrix
%   
%   Usage: 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
%   Author: Ran Wei and Antonio Robles-Kelly
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Z = dist(W, P)

    if nargin < 1 || nargin > 2
        error('Error found in input arguments. Please check.');
    else
        if nargin == 1
            Z = dist_(W);
        else
            Z = dist_(W, P);
        end
    end
    
end