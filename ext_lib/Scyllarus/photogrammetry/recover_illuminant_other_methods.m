% recover_illuminant_other_methods: estimates the light source of an input_image. 
%
% Depending on the parameters the estimation is equal to Grey-Wolrd, White-Patch, 
% Shades-of-Gray or Grey-Edge algorithms.
%
% This code has been revised to be able to process the hyper spectral image
%
% SYNOPSIS:
%    L = recover_illuminant_other_methods(I, options, mask)
%    
% INPUT :
%   I       : Input image cube (cols x rows x bands)
%   options : struct containing the fields relevant to each method
%   mask    : Binary image mask with zeros on image positions which
%                   should be ignored.
% OUTPUT: 
%   L       : Estimated illuminant power spectrum
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2014 All Rights Reserved.
% Author: Antonio Robles-Kelly. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function L = recover_illuminant_other_methods(I, options, mask)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %   Setup the variables
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    I = double(I);
    
    [~, ~, band] = size(I);
    
    if ~isfield(options, 'debug') || isempty(options.debug) || options.debug > 3 || options.debug < 0
        options.debug = 0;
    end

    if exist('mask', 'var')
        for i = 1:band
            I(:, :, i) = I(:, :, i).*mask;
        end
    end
    
    if ~isfield(options, 'drate')
        options.drate = 1;
    end

    if ~exist('options', 'var') || ~isfield(options, 'method')
            options.method = 'GW';    %This is the default, i.e. Gray World
            if options.debug > 0
                disp('Method is not set. Using default method: the Grey World method');
            end
    end
 
    switch upper(options.method)
        case 'GW'
            L = light_gray_world_(I, options.drate);
            if options.debug > 0
                disp('Performing the illuminant power spectrum recovery using the Grey World method.');
            end
        case 'SG'
            if ~isfield(options, 'order')
                options.order = 2;
            end
            L = light_shade_gray_(I, options.drate, options.order);
            if options.debug > 0
                disp('Performing the illuminant power spectrum recovery using the Shade of Grey method.');
            end
        case 'WP'
            L = light_white_patch_(I, options.drate);
            if options.debug > 0
                disp('Performing the illuminant power spectrum recovery using the White Patch method.');
            end
        case '1STOGE'
            if ~isfield(options, 'order')
                options.order = 2;
            end
            if options.debug > 0
                disp('Performing the illuminant power spectrum recovery using the 1st order Grey Edge method.');
            end
            L = light_gray_edge_(I, options.drate, options.order);
        case '2NDOGE'
            if ~isfield(options, 'order')
                options.order = 2;
            end
            if options.debug > 0
                disp('Performing the illuminant power spectrum recovery using the 2nd order Grey Edge method.');
            end
            L = light_gray_edge_(I, options.drate, options.order);
        otherwise
    end
    if options.debug > 0
        disp('illuminant power spectrum recovery is done');
    end

end
