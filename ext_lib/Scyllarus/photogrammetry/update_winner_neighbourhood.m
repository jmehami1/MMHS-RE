%   This function is used to update the neighbourhood in a 3D neuron network around a winner neuron 
% Syntax
%    UPD_N = update_winner_neighbourhood(NEURONS, INPUT, pos, options)
%
% Description
%
%   Update the area around a winner neuron in a som 3D neuron matrix.
%    
% Input:
%
%   NEURONS: the neuron network in 2D form which is of size (bands, length)
%
%   INPUT:   the vector of input which is of size (bands, 1)
%
%   pos:     the position of the winner neuron in matrix, around which
%            other neurons will be updated 
%
%   options: options includes height, width, sigma, learn_rate, radius and threshold. 
%            height, width, depth: dimensions of NEURON
%            sigma and learn_rate: options used to update neurons. by
%                                  default, sigma == 4, learn_rate == 0.75
%            radius:               the radius of updating neurons around the winner
%            threshold:            the threshold of weight below which the
%                                  particular neuron will not be updated.   
%            debug:                Level of debuging information, default
%                                  to 1, max 3
%
% Output:
%   
%   UPD_N:   Updated neuron network
%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Ran Wei
% Version: 1.0.2
% Last Update Date: 22 Sep 2014

function UPD_N = update_winner_neighbourhood(NEURONS, INPUT, pos, options, debug)

    switch nargin
        case 5
            UPD_N = update_winner_neighbourhood_(NEURONS, INPUT, pos, options, debug);
        case 4
            UPD_N = update_winner_neighbourhood_(NEURONS, INPUT, pos, options);
        case 3
            UPD_N = update_winner_neighbourhood_(NEURONS, INPUT, pos);
        otherwise
            error('Incorrect input argument');
    end
   
end