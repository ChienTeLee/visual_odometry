function [inlierIdx, F] = selfEstimateFundamentalMatrixRANSAC(points1, points2, iteration, epsilon)
    % estimate fundamental matrix using RANSAC
    nMaxInlier = 0;
    nPoints = size(points1, 2);
    for i = 1 : iteration
        % sample 8 points
        sampleIdx = randsample(nPoints, 8);

        % use normalized 8-point algorithm to estimate fundamental matrix
        F = selfGetFundamentalMatrix(points1(:,sampleIdx(:)), points2(:,sampleIdx(:)));

        % select inlier features which has error < epsilon
        selectIdx = abs(diag(points2' * F * points1)) < epsilon;
        nInliers = sum(selectIdx);
        
        % update if there are more inlier points
        if nInliers > nMaxInlier
            nMaxInlier = nInliers;
            inlierIdx = selectIdx;
        end
    end
    
    % recalculate F using inlier points
    F = selfGetFundamentalMatrix(points1(:,inlierIdx(:)), points2(:,inlierIdx(:)));
end
