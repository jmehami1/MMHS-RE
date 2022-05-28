%% Scyllarus main routine script.
%
%%  Syntax:
%   HSZ = Scyllarus(I);
%   HSZ = Scyllarus(I, options);
%   HSZ = Scyllarus(filename);
%   HSZ = Scyllarus(filename, options);
%   HSZ = Scyllarus(I, options, Endmembers);
%   HSZ = Scyllarus(I, options, Endmembers, CanonicalIlluminants);
%   HSZ = Scyllarus(HS);
%   HSZ = Scyllarus(HS, options);
%   HSZ = Scyllarus(HS, options, Endmembers);
%   HSZ = Scyllarus(HS, options, Endmembers, CanonicalIlluminants);
%
%%  Description:
% This is the implementation of the Scyllarus pipeline for the dichromatic model, 
% which comprises the recovery of the specularities, shading factor, reflectance
% and illuminant power spectrum.
%
%%  Input:
%
%   I: hyperspectral image stored as a 3D array. 
%   HS: An HSZ data structure (see below)
%   options: Structure with the following fields
%         L: A structure defining the options for the illuminant recovery step
%               bitdepth: Is the data type for the spectral cube, i.e. number of bits per
%                   spectral measurement. By fault this is 16.
%               method: String specifying the method to be used. It can be any of the following
%                       'HRK': Employs the method of Huynh and Robles-Kelly (A Solution of the 
%                           Dichromatic Model for Multispectral Photometric Invariance, 
%                           International Journal of Computer Vision 2010).
%                       'FS': Uses the method of Finlayson and Schaefer (Convex and Non-convex 
%                           Illuminant Constraints for Dichromatic Colour Constancy, CVPR 2001).
%                       'GW': Uses the Grey World method.
%                       'SG': Uses the Shade of Grey method.
%                       'WP': Uses the White Patch method.
%                       '1stOGE': Uses the 1st order Grey Edge method.
%                       '2ndOGE': Uses the 2nd order Grey Edge method.
%               drate: Image downsampling rate for the Grey World, Shade of Grey, 
%                       White Patch and Grey Edge methods. The default is 1, 
%                       i.e. no downsampling.
%               shadeOfGreyOrder: The order of the L^p mean used for the Shade of Grey method.
%                       The default is 1.
%               alpha:   The value for the regularisation term used for the HRK
%                   (Huynh and Robles-Kelly) method. The default for this is 50. 
%               patches: Pre-selected patches. This could be a set of geometry data of patches
%                   with a format of (Top_left_y, top_left_x, height, 
%                   width). This can be left empty.
%               PSFFactor: The factor used for the PSF employed to 'soften' the distribution of abundances for
%                   the illuminants. The default is 10.
%               DEBUG: Defines the level of debugging information shown at execusion time (DEBUG<6).
%                   the default is 3.
%         R: A structure defining the options for the photometric parameter recovery step
%               method: String specifying the method to be used. This can be 'LS'
%                   for the linear least squares solver, 'KL' for the
%                   energy.
%               neigbourhoodsize (optional, used by the LS option): a threshold used to ignore the processing of regions with a small
%                   number of pixels. Those with a number of pixels smaller than or equal to this are considered
%                   diffuse. (default value is 5) ignoreThresh = 5;
%               grayThresh (optional, used by the LS option): a threshold used to determine whether a material is a shade of gray. If
%                   the reflectance spectra within a cluster does not deviate from a uniform spectrum (a flat
%                   line) beyond this threshold, then we will assume that it is a shade of gray and purely
%                   diffuse. (default value is 2) grayThresh = 2;
%               numclusters (option, used by the KL option): the number of clusters used for the K-means, the
%                   default is 20.
%               DEBUG: Defines the level of displaying debugging information. Default is 1, the least
%                   information will be given
%         IndexingMethod: Method used to index to materials. This can be either
%             'DA' (deterministic annealing) or 'KM' (k-means clustering).
%             The default is 'KM'
%         IlluminantEncoding: Determines the manner in which the spectra on HSZ.L.Elements and HSZ.L.Endmembers 
%             is encoded. The default is 'NURBS', but 'GMM' and 'RAW' can
%             also be used.
%         IlluminantIndexed: Determines whether the pixel wise illuminant is indexed to a set of 
%             illuminants "representative" of the lights in the scene. The default is 0 (no indexation)
%         CanonicalIlluminantIndexed: Turns on and off the indexing of the pixel-wise illuminant
%             power spectrum to canonical illuminants. The default is 0 (off).
%         numIlluminants: Number of lights used to represent each pixel. The default value 
%             is min(maxnumIlluminants, bands).
%         maxnumIlluminants: Number of target representative lights in the scene.
%             Note that this must be greater or equal to numIlluminants.  
%             The default value is 1, i.e. a single illuminant in the scene.
%         numCanonicalIlluminants: Number of target cannonical illuminants in which decompose the
%             each representative lights.
%         MaterialEncoding: Determines the manner in which the spectra on HSZ.S.Elements and HSZ.S.Endmembers 
%             is encoded. The default is 'NURBS', but 'GMM' and 'RAW' can
%             also be used.
%         MaterialIndexed: Determines whether the reflectance is to be decomposed into
%             the materials in the scene. The default is 1 (material indexed).
%         EndmemberIndexed: Turns on and off the indexation of the scene materials to the endmembers on
%             the Endmembers matrix.
%         maxnumMaterials: Maximum number of materials used for the indexation 
%             of the image as a whole. Note that this must be greater or equal to 
%             numMaterials. The default value is min(bands, 25).
%         numMaterials: Number of materials used to express the reflectance for each pixel.
%         numEndmembers: Number of endmembers that should be used to express each material.
%         SpecularityEncoding: Encoding scheme used for the highlights at output. 
%             The default is 'NURBS', but 'GMM' and 'RAW' can also be used.
%         SpecularityIndexed: Turn on and off the indexation of the highlights to a statistically
%             consistent set of spectral vectors. The default is 1 (on)
%         maxnumHighlights: Maximum number of spectral vectors used for the
%             indexation of the image as a whole. Note that this must be
%             greater or equal to numHighlights. The default is min(bands, 25).
%         numHighlights: Number of spectral vectors used for the indexation. The default is 1, which, 
%             by definition, sets the output HSZ.K.Elements to a uniform (all ones) spectral
%             vector. This also implies that the highlights are not wavelength dependant.
%         PSFFactor: The factor used for the PSF employed to 'soften' the distribution of abundances for
%             the materials, highlights and illuminants. The default is 10.
%   Endmembers: A structure with the following fields.
%         Endmembers: An array containing the endmember spectra
%         HDR: A structure containing the following fields
%               wavelength: vector of wavelengths.
%               Matxx: Material names
%   CanonicalIlluminants: A structure with the following fields.
%         Endmembers: An array containing the canonical illuminant spectra
%         HDR: A structure containing the following fields
%               wavelength: vector of wavelengths.
%               Matxx: Material names
%
%%  Output:
%
%   HSZ: A structure containing the following fields
%       HDR: A structure containing the fields corresponding to I.HDR and, 
%             in addition, the following fields
%             EncodingL: Encoding scheme for the data on HSZ.L.Elements and HSZ.L.Endmembers. This is 
%                 equivalent to options.IlluminantEncoding
%             IndexedL: Field equivalent to options.IlluminantIndexed.
%             EndmemberIndexedL: Field equivalent to options.CanonicalIlluminantIndexed.
%             numElementsL: Number of lights comprising the illuminant mixture for each pixel. This is, 
%                 ideally, the same as options.numIlluminants.
%             numEndmembersL: Number of cannonical illuminants comprising each of the representative
%                 lights in the scene.
%             degreeNURBSL: Degree of the polynomial used for the spline
%                 function used to represent the illuminant spectra. This has been set to 2.
%             numGMMsL: Number of Gaussians comprising the mixture used to represent 
%                 the illuminant spectra.
%             EncodingS: Encoding scheme for  HSZ.S.Elements and HSZ.S.Endmembers. This is 
%                 equivalent to options.MaterialEncoding.
%             IndexedS: Field equivalent to options.MaterialIndexed.
%             EndmemberIndexedS: Field equivalent to options.EndmemberIndexed.
%             numElementsS: Number of materials comprising each pixel reflectance.
%             numEndmembersS: Number of endmembers comprising each of the materials in the scene.
%             degreeNURBSS: Degree of the polynomial used for the spline
%                 function used to represent the reflectance spectra. This has been set to 2.
%             numGMMsS: Number of Gaussians comprising the mixture used to represent 
%                   the reflectance spectra.
%             EncodingK: Encoding scheme used for the specularities in the image.
%             IndexedK: Turn on and off the indexation of the highlights to a statistically
%                 consistent set of spectral vectors.
%             numElementsK: Number of spectral vectors used for the indexation. 
%             numGMMsK: Number of Gaussians comprising the mixture used to 
%                 represent the highlight spectra. 
%             degreeNURBSK: Degree of the polynomial used for the spline
%                 function used to represent the highlights. This has been set to 2.
%       L: A structure containing one or more of the following fields corresponding to the
%          illuminant.
%             Elements: Spectra for the representative illuminants in the
%             scene. 
%             ElementAbundances: Abundance matrix for the spectra on Elemements.
%             ElementAbundanceIndexes: Index matrix for the abundances of the element spectra. 
%             Endmembers: Spectra for the canonical illuminant library.
%             EndmemberAbundances: Canonical illuminant abundance matrix for the representative
%                 illuminants. This is a step further in the indexation of the spectra.
%             EndmemberAbundanceIndexes: Index matrix for the abundances of the canonical
%                 illuminant spectra.
%       R: A structure containing one or more of the following fields corresponding to the
%          reflectance.
%             Elements: Spectra for the materials in the scene.
%             ElementAbundances: Abundance matrix for the spectra on Elemements.
%             ElementAbundanceIndexes: Index matrix for the abundances of the element spectra. 
%             Endmembers: Spectra for the endmember library.
%             EndmemberAbundances: Endmember abundance matrix for the representative
%                 illuminants. This is a step further in the indexation of the spectra.
%             EndmemberAbundanceIndexes: Index matrix for the abundances of the endmember spectra.
%             Factor: Matrix of pixelwise scaling factors for the reflectance
%       K: A structure containing one or more of the following fields corresponding to the
%          specular highlights
%             Elements: Spectra for the set of basis spectral vectors spanning the
%                  specular highlights. This is only present if IndexedK ==
%                  1.
%             ElementAbundances: Abundance matrix for the spectra on Elemements.
%             ElementAbundanceIndexes: Index matrix for the abundances of the element spectra. 
%             Factor: Matrix of pixelwise specular coefficients.
% 
%%  Examples
% 
% Read an FLA file and process it using Scyllarus with material indexation and 
% 5 materials per pixel
% 
%   I = FLAread('.\shared\samples\apples_small.fla');
%   options = struct('numMaterials', 5, 'DEBUG', 3);
%   HSZ = Scyllarus(I, options);
%   or
%   options = struct('numMaterials', 5, 'DEBUG', 3);
%   HSZ = Scyllarus('.\shared\samples\apples_small.fla', options);
% 
% Read an FLA file rescaling it to half is size and an SLZ file (end-member library file) 
% and process it using Scyllarus with both, material indexation and unmixing.
%
%   I = FLAread('.\shared\samples\apples_small.fla', 0.5);
%   Endmembers = SLZread('ISSA72.slz');
%   options = struct('MaterialIndexed', 1, 'numIlluminants', 3);
%   HSZ = Scyllarus(I, options, Endmembers);
%
% Read an FLA file and an SLZ file (end-member library file) and process it using
% Scyllarus with both, material indexation and unmixing.
%
%   I = FLAread('.\shared\samples\apples_small.fla');
%   Endmembers = SLZread('ISSA72.slz');
%   CanonicalIlluminants = SLZread('ISSAIlluminants9.slz');
%   options = struct('MaterialIndexed', 1, 'numIlluminants', 3);
%   HSZ = Scyllarus(I, options, Endmembers, CanonicalIlluminants);
%   HSZwrite('.\shared\samples\face.hsz', HSZ);
%
% Read an FLA file and an SLZ file (end-member library file), process it using
% Scyllarus with both, material indexation and unmixing and encode it using 
% Gaussian mixtures.
%
%   I = FLAread('.\shared\samples\apples_small.fla', 500, 620);
%   Endmembers = SLZread('XRiteCC154.slz');
%   CanonicalIlluminants = SLZread('ISSAIlluminants9.slz');
%   options = struct('MaterialIndexed', 1, 'numIlluminants', 3, 'MaterialEncoding', ...
%                    'GMM', 'SpecularityEncoding', 'GMM', 'IlluminantEncoding', 'GMM');
%   HSZ = Scyllarus(I, options, Endmembers, CanonicalIlluminants);
%   Irgb = recover_RGBImage(HSZ);
%
% Read an FLA file and process it using Scyllarus without material indexation and 
% RAW output by recovering a single illuminant with the generalised gray
% world method ('GWW'). Once the HSZ is computed, the radiance with the
% specularities, shading and illuminant removed is visualised as a trichromatic
% image and the global illuminant in the scene is ploted.
%
%   I = FLAread('.\shared\samples\apples_small.fla');
%   options = struct('MaterialIndexed', 0, 'SpecularityIndexed', 0, 'MaterialEncoding', ...
%                    'RAW', 'SpecularityEncoding', 'RAW', 'IlluminantEncoding', 'RAW', 'numIlluminants', 1);
%   options.L.method = 'GWW';
%   HSZ = Scyllarus(I, options);
%   I.I = HSZ.S.Elements;
%   Irgb = recover_RGBImage(I);
%   imtool(Irgb); 
%   plot(HSZ.HDR.wavelength, HSZ.L.Elements);
%
%
%%  See also
%       recover_RGBImage, HSZwrite, HSZread, SLZread, FLAread 
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  This computer code is subject to copyright: (c) National ICT Australia Limited (NICTA) 2013-2014 All Rights Reserved.
%  Author: Ran Wei and Antonio Robles-Kelly. 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function HSPipeline = Scyllarus(I, options, Endmembers, CanonicalIlluminants)

    switch nargin
        case 1
            HSPipeline = Scyllarus_(I);
        case 2
            HSPipeline = Scyllarus_(I, options);
        case 3
            HSPipeline = Scyllarus_(I, options, Endmembers);
        case 4
            HSPipeline = Scyllarus_(I, options, Endmembers, CanonicalIlluminants);
        otherwise
            error('Incorrect input arguments');
    end
    
end

