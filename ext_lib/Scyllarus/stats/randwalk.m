% Syntax
%
%   [Labels, Prob] = randwalk(G);
%
% Description:
%   Implementation of the generalised random walk with side information for labelling as presented by Z. Fu and A Robles-Kelly in the paper
%   entitled "A Quadratic Programming Approach to Image Labeling" (IET Computer Vision, 2008).
% 
% Input: 
% 
%   G: Structure containing the graph. Note that idxCls may be empty (this allows for the use of I2Graph with a fixed
%      number of clusters (G.nCls). This structure includes the following members:
%
%       width, height: width and height of the input image.
%       ii, jj, vv:    Triplet for the graph edge term. These is such that the weighted adjacency matrix of the graph is
%                      given by W = sparse(G.ii, G.jj, G.vv, N, N), where N=G.width*G.height;
%       e:             Data term with the probability of the vertex belonging to the cluster. This is an N x c array
%                      where N is the number of vertices as before and c the number of classes.
%       idxCls:        Cell of order c (number of classes) where idxCls{c} contains the indeces for the labelled pixels
%                      in the class c. 
%       mCls:          Mean vectors for the labelled pixels. This is a nCls x bands array where the row index
%                      corresponds to the class. 
%       nCls:          Number of labelled classes in the mask file.
% 
% Output: 
%
%   LABELS: Index of the most likely label for each entry entry.
%   PROB:   Vector whose entry indexed i, c denotes the probability of the node i belonging to the class c. 
%   
%
% Example:
%
%       imdata = imread('ngc6543a.jpg');
%       mask = imread('ngc6543a_mask.tif');
%       G = Mask2Graph(imdata, mask);
%       [Labels, Prob] = randwalk(G);
%       imtool(reshape(Labels, G.height, G.widthh)/G.nCls);
%
%   See also:
%
%       I2Graph, Data2Graph
%
%   Notes: 
%
%   This routine requires the SuiteSparse package, which can be downloaded from 
%   http://www.cise.ufl.edu/research/sparse/SuiteSparse/
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014-2015 All Rights Reserved.
%  Author: Zhouyu Fu, Antonio Robles-Kelly and Ran Wei
%   Version: 1.0.3
%   Last update date: 19 June 2015


function [LABELS, PROB] = randwalk(G)

    [LABELS, PROB] = randwalk_(G);

end