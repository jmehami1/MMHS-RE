% Mix end-members so as to obtain mixed spectra.
%
% Syntax
%     Q = mix_spectra(Abundances, Indexes, Endmembers)
%     Q = mix_spectra(Abundances, Indexes, Endmembers, numMaterials)
%     Q = mix_spectra(Abundances, Indexes, Endmembers, numMaterials, normalise)
% 
% Description
%
% Mix the spectra given a set of abundances, indexes and endmembers. 
% This is, effectively, the inverse operation to unmixing.
%
% Inputs:
%
%     Abundances:   Matrix containign the abundance coefficients per endmember. This is a 
%                   cols x rows x endmembers matrix, where the output matrix Q is cols x rows x bands.
%     Indexes:      Matrix containing the indexes for the endmembers.
%     Endmembers:   Matrix whose rows are indexed to the endmembers and columns to the 
%                   wavelength, i.e. Endmember(i,:) contains the ith endmember spectrum
%     numMaterials: Number of materials used for the recovery of the spectra. If 
%                   this is not provided, the size of 3rd dimension of the abundance matrix is used.
%     normalise:    If normalise is unity, the output spectra is normalised accordingly, 
%                    i.e. norm(Q(i,i),3) is equivalent to 1. The default is 1.
% 
% Outputs:
%    Q: Matrix of mixed spectra
% 
% See also
%   L2_unmixing
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013 All Rights Reserved.
% Author: Ran Wei and Antonio Robles-Kelly.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Q = mix_spectra(Abundances, Indexes, Endmembers, numMaterials, normalise)

    switch nargin
        case 5
            Q = mix_spectra_(Abundances, Indexes, Endmembers, numMaterials, normalise);
        case 4
            Q = mix_spectra_(Abundances, Indexes, Endmembers, numMaterials);
        case 3
            Q = mix_spectra_(Abundances, Indexes, Endmembers);
        otherwise
            error('Please check input arguments.');
    end
    
end

