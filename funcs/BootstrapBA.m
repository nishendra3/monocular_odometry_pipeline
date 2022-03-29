function [KLT1, KLTCand, gData, CKpts] = BootstrapBA(I1, I2, K, gData)
% Bootstraping the Pipeline.

control_script;
parameter_initialization;

% TODO 1: use a machine learning detector ( can not find one that returns
% points based detectors )
% TODO 2: seems like ORB combines many good staff sto try it for sure

% if re-bootstrap is used with just passing one image
if nargin==1
    KLT1=[];KLTCand=[];gData=[];
    if Harris
        CKpts = detectHarrisFeatures(I1, 'MinQuality', harris_min_quality, 'FilterSize', harris_filter_size);         
    elseif FAST
        CKpts = detectFASTFeatures(I1,'MinQuality', fast_min_quality, 'MinContrast', fast_min_contrast);
    elseif ORB
        CKpts = detectORBFeatures(I1, 'ScaleFactor', ORB_scale_factor, 'NumLevels', ORB_num_levels);
    else
        CKpts = detectSIFTFeatures(I1);           
    end
    CKpts = CKpts.Location;
    return
end
CKpts = [];

% In control_script the Harris is used and has been tuned for the datasets
if Harris
    points1 = detectHarrisFeatures(I1, 'MinQuality', harris_min_quality, 'FilterSize', harris_filter_size);         
    points2 = detectHarrisFeatures(I2, 'MinQuality', harris_min_quality, 'FilterSize', harris_filter_size); 
elseif FAST
    points1 = detectFASTFeatures(I1,'MinQuality', fast_min_quality, 'MinContrast', fast_min_contrast);
    points2 = detectFASTFeatures(I2,'MinQuality', fast_min_quality, 'MinContrast', fast_min_contrast);
elseif ORB
    points1 = detectORBFeatures(I1, 'ScaleFactor', ORB_scale_factor, 'NumLevels', ORB_num_levels);
    points2 = detectORBFeatures(I2, 'ScaleFactor', ORB_scale_factor, 'NumLevels', ORB_num_levels);
else
    points1 = detectSIFTFeatures(I1);        
    points2 = detectSIFTFeatures(I2);   
end


if KLT_check  
    KLT = vision.PointTracker('NumPyramidLevels', klt_NumPyramidLevels, ...
                              'MaxBidirectionalError', klt_MaxBidirectionalError, ...
                              'BlockSize', klt_BlockSize , ...
                              'MaxIterations', klt_MaxIterations);
    
    initialize(KLT, points1.Location, I1);
    [points_klt,kp_validity] = step(KLT,I2);

    %convert to cornerPoint object to match template 
    if Harris || FAST   
        points2 = cornerPoints(points_klt);
    elseif ORB
        points2 = ORBPoints(points_klt);
    end

    matchedPoints1 = points1(kp_validity);
    matchedPoints2 = points2(kp_validity);

else %not efficient
    [features1,valid_points1] = extractFeatures(I1,points1);
    [features2,valid_points2] = extractFeatures(I2,points2);
    
    indexPairs = matchFeatures(features1,features2);
    
    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:);
end


%P3PRansac equivalent of Matlab
[F, inliers] = estimateFundamentalMatrix(matchedPoints1.Location, matchedPoints2.Location, 'Method', 'RANSAC', 'NumTrials',2000);

cameraParams = cameraParameters('IntrinsicMatrix', K');

InlierPoints1 = matchedPoints1(inliers,:);
InlierPoints2 = matchedPoints2(inliers,:);

 % Disambiguation of Fundamental Matrix to ge tthe Relative poses
[relOrien, relTrans] = relativeCameraPose(F, cameraParams, InlierPoints1, InlierPoints2);
[R, t] = cameraPoseToExtrinsics(relOrien, relTrans);

% Triangulate the points
M1 = cameraMatrix(cameraParams, eye(3), zeros(1,3));
M2 = cameraMatrix(cameraParams, R, t);
[Points3D, ~, validIndx] = triangulate(InlierPoints1, InlierPoints2, M1 ,M2);

% Take points ahead of camera
Points3D = Points3D(validIndx, :);
InlierPoints1 = InlierPoints1(validIndx, :);
InlierPoints2 = InlierPoints2(validIndx, :);


% % Create point cloud
% plotPC(Points3D(Points3D(:,3)<500,:), relTrans, relOrien)

% Add pose data to global dataset
gData.vSetKp = addView(gData.vSetKp, 1, rigid3d(eye(3), zeros(1,3)), "Points", InlierPoints1);
gData.vSetKp = addView(gData.vSetKp, 2, rigid3d(relOrien, relTrans), "Points", InlierPoints2);
gData.vSetKp = addConnection(gData.vSetKp, 1, 2 , 'Matches', [1:size(InlierPoints1,1); 1:size(InlierPoints1,1)]');

[gData.wpSet, newPointIdx] = addWorldPoints(gData.wpSet, Points3D);
pointIndices = 1:size(Points3D, 1);
featureIndices = 1:size(InlierPoints1);
gData.wpSet = addCorrespondences(gData.wpSet, 1, pointIndices, featureIndices);
gData.wpSet = addCorrespondences(gData.wpSet, 2, pointIndices, featureIndices);
 

% Calculate candidate keypoints (all the points that are not inliers and not valid from KLT)
ckpoints1 = matchedPoints1(inliers == 0,:);
ckpoints2 = matchedPoints2(inliers == 0,:);
gData.vSetCkp = addView(gData.vSetCkp, 1, rigid3d(eye(3), zeros(1,3)), "Points", ckpoints1);
gData.vSetCkp = addView(gData.vSetCkp, 2, rigid3d(relOrien, relTrans), "Points", ckpoints2 , "Features", horzcat(relOrien(:)'.*ones(size(ckpoints2,1),1), relTrans.*ones(size(ckpoints2,1),1), ckpoints2.Location));
gData.vSetCkp = addConnection(gData.vSetCkp, 1, 2 , 'Matches', [1:size(ckpoints1,1); 1:size(ckpoints1,1)]');


% Run bundleadjust
tracks       = findTracks(gData.vSetKp);
cameraPoses  = poses(gData.vSetKp);
intrinsics = cameraIntrinsics(cameraParams.FocalLength,cameraParams.PrincipalPoint,size(I1));
[refinedPoints, refinedAbsPoses] = bundleAdjustment(Points3D, tracks, ...
    cameraPoses, intrinsics, 'FixedViewIDs', 1, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 20);


gData.vSetKp = updateView(gData.vSetKp , refinedAbsPoses);
gData.wpSet  = updateWorldPoints(gData.wpSet, newPointIdx, refinedPoints);


% setup KLT trackers- 
KLT1 = vision.PointTracker('NumPyramidLevels', klt_NumPyramidLevels, ...
                                'MaxBidirectionalError', klt_MaxBidirectionalError, ...
                                'BlockSize', klt_BlockSize, ...
                                'MaxIterations', klt_MaxIterations);

initialize(KLT1, InlierPoints2.Location, I2);

KLTCand = vision.PointTracker('NumPyramidLevels', klt_NumPyramidLevels, ...
                                'MaxBidirectionalError', klt_MaxBidirectionalError, ...
                                'BlockSize', klt_BlockSize, ...
                                'MaxIterations', klt_MaxIterations);
initialize(KLTCand, ckpoints2.Location, I2);

end