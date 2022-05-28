%   Syntax:
%       function SLZ = mat2slz(MAT, HDR, normalise)
%
%   Description:
%       this function is used to convert a matrix into a SLZ library 
%
%   Input:
%       MAT: matrix to be converted. size: numEndmember column vectors
%           (each of them has bands elements)
%       Labels: is a string vector containing the names of each library
%               vector. By default, it values will be 'mat1', 'mat2', ...,
%               'matn' where n is the number of library vectors. It should
%               contain the same number of elements as that of library
%               vectors.
%       HDR: header information required. It should contains the following
%            information (but not limited to): wavelength, wavelength_unit
%            and numEndmembers. If HDR is missing, this function will
%            use default values such as wavelength = [number_of_bands, 1]
%            with value starting from 400 to 700 and increase by 10.
%            wavelength unit 'Nanometers'. numEndmembers will be the number
%            of library vectors.
%       normalise: whether normalise input value. Default value is 1 (yes).
%
%   Output
%       SLZ: result in SLZ format

function SLZ = mat2slz(MAT, Labels, HDR, normalise)
    
    switch nargin
        case 4
            SLZ = mat2slz_(MAT, Labels, HDR, normalise);
        case 3
            SLZ = mat2slz_(MAT, Labels, HDR);
        case 2
            SLZ = mat2slz_(MAT, Labels);
        case 1
            SLZ = mat2slz_(MAT);
        otherwise
            error('Error in input arguments');
    end
    
end