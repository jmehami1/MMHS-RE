% Syntax
%       G = mask2graph(I, Mask);
%       G = mask2graph(I, Mask, options);
%       
% Description:
%       Computes the weighted adjacency matrix for the image cube I.
% 
% Input: 
%       I:      the radiance image (stored as a 3D array with size height x width x bands).
%       Mask:   Image labels (stored as a 2D numerical array of size height x
%               width); Unlabled pixels should be either white or black.
%       options: a structure containing the following fields
%           nn: Neigbhourhood to be used to build the adjacency matrix. For
%               instance, if nn=2, a neighbourhood of 2 pixels about.
%           dist: Type of distance used. This can be set to 'Angular', i.e. 
%               spectral angle, or 'Euclidean' (L2 norm). The default is
%               'Euclidean'. For the Euclidean distance, the additional value
%               tau has to be set (by default, tau = 1).
%           sigma: constant used to weight the colour/spectra term on the
%               weight matrix. Note the weights denote similarity, not
%               distance. This is, a high value of the weight between two nodes
%               implies these are "close" to each other. The default value is 1.
%           gamma: Constant used to weight the spatial term. The default value is 1.
%           fhandle: Function handle. This has been provided so as to allow the
%               definition of custom data terms G.e(i,c), where i is the node index
%               and c is the class prototype index. Use the function handle @ to
%               set options.fhandle accordingly. If none is specified, the data
%               term is given by the Euclidean angle or the L2 norm.
% 
% Output: 
%       G: Structure containing the graph model. It contains the following fields
%           width, height: width and height of the input image.
%           ii, jj, vv:    Triplet for the graph edge term. These is such that the weighted adjacency matrix of the
%                          graph is given by W = sparse(G.ii, G.jj, G.vv, N, N), where N = G.width*G.height;
%           e:             Data term with the probability of the vertex belonging to the cluster. This is an N x c array
%                          where N is the number of vertices as before and c the number of classes.
%           idxCls:        Cell of order c (number of classes) where idxCls{c} contains the indeces for the labelled
%                          pixels in the class c. 
%           mCls:          Mean vectors for the labelled pixels. This is a nCls x bands array where the row index
%                          corresponds to the class. 
%           nCls:          Number of labelled classes in the mask file.
%
% Example:
%
%       imdata = imread('ngc6543a.jpg');
%       mask   = imread('ngc6543a_mask.tif');
%       options.nn = 4;
%       G = mask2graph(imdata, mask, options);
%
%   See also:
%
%       I2graph, data2graph
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly

function G = mask2graph(I, Mask, options)

    if ~exist('options', 'var')
        options = [];
    end;
    
    G = mask2graph_(I, Mask, options);   

end

