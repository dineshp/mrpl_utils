classdef MatrixFxns 
    methods (Static)
        function bool = matrixDimensionEqual(A, B)    
        
            bool = isequal(size(A), size(B)) || (isvector(A) && isvector(B) && numel(A) == numel(B));
        end
    end
end