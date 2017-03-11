function [ R ] = zeroElimMeanFilter( I )
    rows = size(I, 1);
    cols = size(I, 2);
    R = zeros(rows, cols, class(I));
    I = hhf_pad(I);
   
    i = (-2:2);
    j = i;
    for m = 0:rows-1
        for n = 0:cols-1
            A = I(m+i+3, n+j+3);
  
            nz = nnz(A);
            if(nz == 0)
                R(m+1, n+1) = 0;
            else
                R(m+1, n+1) = sum(A(:)) / nz;
            end
        end
    end
end

