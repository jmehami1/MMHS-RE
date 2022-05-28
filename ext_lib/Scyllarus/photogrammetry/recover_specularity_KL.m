%  Syntax:
%   K = recover_specularity_KL(I, L, cluster_num, debug)
%   
%  Description:
%   Recover the specularity map from a single hyperspectral image. The method used
%   in this function was described in DICTA Paper `Specularity Removal from Imaging Spectroscopy
%   Data via Entropy Minimisation' by Lin Gu and Antonio Robles-Kelly
%      
%  Input: 
%
%       I:           Input image cube, can be in either multi-spectrum or RGB image
%       L:           Light spectrum vector.
%       cluster_num: The number of clusters used for the K-means algorithm
%       debug:       Debugging information display level (1 - 3). 
%
%  Output: 
%
%       K:           the map of specular coefficients at the image pixels,
%                    stored as a 2D array of size height x width.  
%
%
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 - 2014 All Rights Reserved.
% Author: Ran Wei, Lin Gu and Antonio Robles-Kelly. 

function K = recover_specularity_KL(I, L, cluster_num, debug)
    
    switch nargin
        case 4
            K = recover_specularity_KL_(I, L, cluster_num, debug);
        case 3
            K = recover_specularity_KL_(I, L, cluster_num);
        case 2
            K = recover_specularity_KL_(I, L);
        otherwise
            error('Scyllarus:photogrammetry:recover_sepcularity - Incorrect input arguments')
    end

end