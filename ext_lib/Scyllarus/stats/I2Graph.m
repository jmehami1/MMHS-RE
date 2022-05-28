%  Syntax
%       G = I2graph(I);
%       G = I2graph(I, options);
%       
%  Description:
%       Computes the weighted adjacency matrix for the image cube I.
% 
%  Input: 
%       I:       the radiance image (stored as a 3D array with size height x width x bands).
%       options: a structure containing the following fields
%                nn:    Neigbhourhood to be used to build the adjacency matrix. For instance, if nn = 2, a neighbourhood
%                       of 2 pixels about. 
%                sigma: constant used to weight the color/spectra term on the weight matrix. Note the weights denote
%                       similarity, not distance. This is, a high value of the weight between two nodes implies these
%                       are "close" to each other. The default value is 1. 
%                gamma: Constant used to weight the spatial term. The default value is 1.
% 
%  Output: 
%       G: Structure containing the graph model. It contains the following fields
%           width, height: width and height of the input image.
%           ii, jj, vv:    Triplet for the graph edge term. These are such that the weighted adjacency matrix of the
%                          graph is given by W = sparse(G.ii, G.jj, G.vv, N, N), where N = G.width*G.height;
%
%  Example:
%
%       imdata = imread('./shared/samples/ngc6543a.jpg');
%       G = I2graph(imdata);
%
% This computer code is subject to copyright: (bands) National ICT Australia Limited (NICTA) 2014-2015 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function G = I2Graph(I, options)

    if nargin == 2
        G = I2Graph_(I, options);
    else
        G = I2Graph_(I);
    end
    
end