function [R, T] = selfReconstructRTMatrix(F, K, points1, points2)
    % get essential matrix from fundamental matrix and camera matrix
    E = K' * F * K;
    % enforce essential matrix to be rank = 2
    [U, ~, V] = svd(E);
    D = diag([1 1 0]);
    E = U * D * V';
    [U, ~, V] = svd(E);

    % get Rotation and Translation  matrix
    W = [0  -1   0;
         1   0   0;
         0   0   1];
    
    R1 = U * W  * V';
    R2 = U * W' * V';
    T1 =  U(:, 3);
    T2 = -U(:, 3);
    
    % correct det(rotation) = -1 by reflection
    if det(R1) < 0
        R1 = -R1;
    end
    if det(R2) < 0
        R2 = -R2;
    end
    
    % 4 solutions with R T
    RTSolutions(:,:,1) = [R1, T1];
    RTSolutions(:,:,2) = [R1, T2];
    RTSolutions(:,:,3) = [R2, T1];
    RTSolutions(:,:,4) = [R2, T2];
    
    % get feature point position on image
    x1 = inv(K) * points1;
    x2 = inv(K) * points2;

    nPoints = size(points2, 2);
    nValid = zeros(1,4);
    
    % x1, x2 are feature point in image1, image2, X is feature point in 3D
    % (x = [x, y, 1], and X = [Y, Y, Z, 1])
    % x1 = P1 * X and x2 = P2 * X  
    % solve x กั (PX) = 0 (which means A * X = 0) for 4 R T solutions
    P1 = [eye(3), zeros(3,1)]; % camera projection 1
    for i = 1 : 4
        P2 = RTSolutions(:,:,i); % camera projection 2
        for j = 1 : nPoints
            A = [ x1(1,j)*P1(3,:) - P1(1,:);
                  x1(2,j)*P1(3,:) - P1(2,:); 
                  x2(1,j)*P2(3,:) - P2(1,:);
                  x2(2,j)*P2(3,:) - P2(2,:); ];
            [~, ~, V] = svd(A);
            
            % find X which is feature point in 3D
            % find x2_3d which is x2 image feature point in 3D
            X = V(:,4);
            X = (X / X(4))';  % normalize X(4) to 1
            x2_3d = X(1:3) * P2(:,1:3)' + P2(:,4)';
            % check if X depth (Z-axis) and x2_3d depth (Z-axis) both > 0
            if (X(3) > 0 && x2_3d(3) > 0)
                nValid(i) = nValid(i) + 1;
            end
        end
    end

    [~, idx] = max(nValid);
    RT = RTSolutions(:,:,idx);
    R = RT(:, 1:3);
    T = RT(:, 4);
end
