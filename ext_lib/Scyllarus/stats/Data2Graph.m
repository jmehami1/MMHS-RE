% Syntax
%       G = data2graph(Data);
%       G = data2graph(Data, options);
%       
% Description:
%       Computes the weighted adjacency matrix for the 2D image matrix I.
% 
% Input: 
% 
%   Data: the data matrix (stored as a 2D array with size n x m, where n is the number of data points and m is their
%         dimensionality). 
%   options: a structure containing the following fields
%            nn: Neigbhourhood to be used to build the adjacency matrix. For instance, if nn = 2, a neighbourhood of 2
%                pixels about. 
%            sigma: constant used to weight the data term on the weight matrix. Note the weights denote similarity, not
%                   distance. This is, a high value of the weight between two nodes implies these are "close" to each
%                   other. The default value is 10.
% 
%   Output: 
%
%   G: Structure containing the graph model. It contains the following fields: 
%      w, h: width and height of the input image. 
%      ii, jj, vv: Triplet for the graph edge term. These is such that the weighted adjacency matrix of the graph is
%                  given by W = sparse(G.ii, G.jj, G.vv, N, N),  where N = G.w*G.h;
%
%   Example:
%
%       load fisheriris
%       xdata = meas(51:end, 3:4);
%       G = data2graph(xdata);
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly


function G = data2graph(I, options)

    switch nargin
        case 2
            G = data2graph_(I, options);
        case 1
            G = data2graph_(I);
        otherwise
            error('Incorrect input arguments');
    end

end