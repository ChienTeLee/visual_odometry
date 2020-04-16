function F = selfGetFundamentalMatrix(points1, points2)
    % use normalized 8-point algorithm to estimate fundamental matrix
    % normalize feature point coordinates between -1 and 1 based on image size
    N = [2/1280   0      -1;
         0        2/960  -1;
         0           0    1;];
    
    normPoints1 = N * points1;
    normPoints2 = N * points2;
    
    % solve A * f = 0
    nPoints = size(points1, 2);
    A = [ normPoints2(1,:)'.*normPoints1(1,:)'  normPoints2(1,:)'.*normPoints1(2,:)'  normPoints2(1,:)' ...
          normPoints2(2,:)'.*normPoints1(1,:)'  normPoints2(2,:)'.*normPoints1(2,:)'  normPoints2(2,:)' ...
          normPoints1(1,:)'                     normPoints1(2,:)'                     ones(nPoints, 1);    ];
    
    % f is the eigenvector corresponding to smallest eigenvalue
    [~, ~, V] = svd(A);
    f = V(:,end);
    
    % enforce rank of F to 2
    tmpF = reshape(f, 3, 3)';
    [U, D, V] = svd(tmpF);
    D(3, 3) = 0;
    normF = U * D * V';
    
    % denormalize to get real F
    F = N' * normF * N;
end