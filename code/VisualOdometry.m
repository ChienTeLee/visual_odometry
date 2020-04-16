% Visual Odometry

% Feature-based reconstruction on scene geometry and camera motion.
% Extract SURF features and apply RANSAC method to estimate fundamental matrix.
% Use fundamental matrix to retrieve camera pose and plot 2D motion path.

% self method means self implemented method
% matlab method means using matlab library


% clear workspace
clear;
close all;

% create path for input image
d = dir('../input/stereo/centre/1399381*.png');
filenames = {d.name};
nImages = size(filenames, 2);

% get camera calibration matrix K
[fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel('../input/stereo/centre', '../input/model');
K = [ fx  0   cx;
      0   fy  cy;
      0   0   1 ; ];

% create camera pose (position and angle) for self method and matlab method
selfCameraPosition = [0 0 0]';
selfCameraAngle = eye(3);
matlabCameraPosition = [0 0 0]';
matlabCameraAngle = eye(3);


disp('Start Processing');

figure();
hold on;
for i = 1 : nImages-1
    % Part 0: Preporcess current image and next image
    % preprocess images and find match points
    % preprocess next image
    nextRawImg = imread( fullfile('../input/stereo/centre', char(filenames(i+1))) );
    nextDemosaicedImg = demosaic(nextRawImg,'gbrg');
    nextUndistortedImg = UndistortImage(nextDemosaicedImg, LUT);
    nextDenoisedImg= imgaussfilt(nextUndistortedImg, 1.5);
    nextGrayImg = rgb2gray(nextDenoisedImg);
    nextImg = adapthisteq(nextGrayImg);
    
    % preprocess current image
    if ~exist('curUndistortedImg', 'var')
        curRawImg = imread(fullfile('../input/stereo/centre', char(filenames(i))) );
        curDemosaicedImg = demosaic(curRawImg, 'gbrg');
        curUndistortedImg = UndistortImage(curDemosaicedImg, LUT);
    end
    curHistMatchImg = imhistmatch(curUndistortedImg, nextUndistortedImg); % match the histogram of current image to next image
    curDenoisedImg = imgaussfilt(curHistMatchImg, 1.5);
    curGrayImg = rgb2gray(curDenoisedImg);
    curImg = adapthisteq(curGrayImg);

    % detect and extract SURF feature points
    points1 = detectSURFFeatures(curImg);
    points2 = detectSURFFeatures(nextImg);
    [features1, validPoints1] = extractFeatures(curImg, points1);
    [features2, validPoints2] = extractFeatures(nextImg, points2);
    
    % match feature points
    indexPairs = matchFeatures(features1, features2, 'MatchThreshold', 5.0);
    matchPoints1 = validPoints1(indexPairs(:,1));
    matchPoints2 = validPoints2(indexPairs(:,2));
    
    % reduce match points by setting maximum 500 points
    nMaxPoint = min(size(matchPoints1,1), 500);
    matchPoints1 = matchPoints1(1:nMaxPoint);
    matchPoints2 = matchPoints2(1:nMaxPoint);
    
    
    % Part 1: self RANSAC method
    % extract match points
    selfMatchPoints1 = matchPoints1.Location;
    selfMatchPoints2 = matchPoints2.Location;
    selfMatchPoints1(:,3) = 1;
    selfMatchPoints2(:,3) = 1;
    selfMatchPoints1 = selfMatchPoints1';
    selfMatchPoints2 = selfMatchPoints2';
    
    % use self RANSAC method to estimate fundamental matrix and find inlier index
    iteration = 1000;
    epsilon = 1e-3;
    [inlierIdx, F] = selfEstimateFundamentalMatrixRANSAC(selfMatchPoints1, selfMatchPoints2, iteration, epsilon);
    
    selfInlierPoints1 = selfMatchPoints1(:,inlierIdx(:));
    selfInlierPoints2 = selfMatchPoints2(:,inlierIdx(:));
    
    % show self RANSAC match features in current image and next image (for debugging)
    %figure;
    %showMatchedFeatures(curImg, nextImg, [selfInlierPoints1(1,:); selfInlierPoints1(2,:)]', [selfInlierPoints2(1,:); selfInlierPoints2(2,:);]', 'montage');
    
    % get F E R T matrices
    [R, T] = selfReconstructRTMatrix(F, K, selfInlierPoints1, selfInlierPoints2);
    
    % reconstruct motion
    selfT = -T' * R;
    selfR = R';
    
    selfCurPosition = selfCameraPosition;
    selfNextosition = selfCurPosition + selfCameraAngle * selfT';
    selfCameraPosition = selfNextosition;
    selfCameraAngle = selfR * selfCameraAngle;
    
    %plot camera motion in x z plane
    plot([selfCurPosition(1) selfNextosition(1)], [selfCurPosition(3) selfNextosition(3)], 'r', 'LineWidth', 2);
    
    
    % Part 2: matlab method
    % estimate fundamental matrix
    cameraParams = cameraParameters('IntrinsicMatrix', K'); 
    
    [matlabF, matlabInlierIdx] = estimateFundamentalMatrix(matchPoints1, matchPoints2, 'Method', 'RANSAC', 'NumTrials', 1000, 'DistanceThreshold', 1e-3);
    
    matlabInlierPoints1 = matchPoints1(matlabInlierIdx);
    matlabInlierPoints2 = matchPoints2(matlabInlierIdx);
    
    % show matlab RANSAC match features in current image and next image (for debugging)
    %figure();
    %showMatchedFeatures(curImg, nextImg, matlabInlierPoints1', matlabInlierPoints2', 'montage');
    
    
    % retrieve camera pose
    [matlabR, matlabT] = relativeCameraPose(matlabF, cameraParams, matlabInlierPoints1, matlabInlierPoints2);
    
    % fix extra dimension error
    if ndims(matlabR) == 3
        matlabR = matlabR(:,:,1);
    end
    if size(matlabT, 1) > 1
        matlabT = matlabT(1,:);
    end
    
    % reconstruct motion
    matlabCurPosition = matlabCameraPosition;
    matlabNextPosition = matlabCurPosition + matlabCameraAngle * matlabT';
    matlabCameraPosition = matlabNextPosition;
    matlabCameraAngle = matlabR' * matlabCameraAngle;
    
    % plot camera position on x z plane
    plot([matlabCurPosition(1) matlabNextPosition(1)], [matlabCurPosition(3) matlabNextPosition(3)], 'b', 'LineWidth', 2);
    
    % move on to next image
    curUndistortedImg = nextUndistortedImg;
    disp(['processing image', num2str(i)]);
end

title('Camera Motion')
legend('self method','matlab method')
xlabel('x');
ylabel('y');

% save figure in output directory
mkdir ../output/
savefig('../output/output.fig')

disp('Finish Processing');


