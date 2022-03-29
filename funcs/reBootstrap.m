% function [KLT1, KLTCand, gData, CKpts] = reBootstrap(I1, I2, K, gData)
function [KLT1, KLTCand, gData] = reBootstrap(i, I1, I2, KLT1, KLTCand , K, gData) 
% TODO: write what the function does

run("control_script.m")
run("parameter_initialization.m")

[~,~,~,points1] = BootstrapBA(I1); 

% TODO: All KLT names should be changed
 
    
KLT = vision.PointTracker('NumPyramidLevels', klt_NumPyramidLevels, ...
                          'MaxBidirectionalError', klt_MaxBidirectionalError, ...
                          'BlockSize', klt_BlockSize , ...
                          'MaxIterations', klt_MaxIterations);

initialize(KLT, points1, I1);
[points_klt,kp_validity] = step(KLT,I2);
kp_validity(points_klt(:,1)<0 | points_klt(:,2)<0) = 0;

matchedPoints1 = points1(kp_validity,:);
matchedPoints2 = points_klt(kp_validity,:);

%convert to cornerPoint object to match template 
if Harris || FAST
    matchedPoints1 = cornerPoints(matchedPoints1);
    matchedPoints2 = cornerPoints(matchedPoints2);
elseif ORB
    matchedPoints1 = ORBPoints(matchedPoints1);
    matchedPoints2 = ORBPoints(matchedPoints2);
end

[F, inliers] = estimateFundamentalMatrix(matchedPoints1.Location, matchedPoints2.Location, 'Method', 'RANSAC', 'NumTrials',2000);
cameraParams = cameraParameters('IntrinsicMatrix', K');

InlierPoints1 = matchedPoints1(inliers,:);
InlierPoints2 = matchedPoints2(inliers,:);

camPose1 = poses(gData.vSetKp, i-2).AbsolutePose; % rot,trans of 2nd last frame 
[relOrien, relTrans] = relativeCameraPose(F, cameraParams, InlierPoints1, InlierPoints2);

% Absolute pose - 
% A = horzcat(camPose1.Rotation, camPose1.Translation');
% A = vertcat(A, [0 0 0 1]); % 4x4
% B = horzcat(relOrien,relTrans'); 
% B = vertcat(B, [0 0 0 1]); % 4x4
% C = A*B;
% absOrien = C(1:3,1:3);
% absTrans = C(1:3, 4)';

absTrans = camPose1.Translation' + camPose1.Rotation * relTrans';
absTrans = absTrans';
absOrien = relOrien*camPose1.Rotation;


[R1, t1] = cameraPoseToExtrinsics(camPose1.Rotation, camPose1.Translation);
[R2, t2] = cameraPoseToExtrinsics(absOrien, absTrans);
% triangulate the points
M1 = cameraMatrix(cameraParams, R1, t1);
M2 = cameraMatrix(cameraParams, R2, t2);

[Points3D, ~, validIndx] = triangulate(InlierPoints1, InlierPoints2, M1 ,M2);
% Take points ahead of camera
Points3D = Points3D(validIndx, :);
InlierPoints1 = InlierPoints1(validIndx, :);
InlierPoints2 = InlierPoints2(validIndx, :);

% remove is close to existing features --
% query = InlierPoints2.Location;
% neighbors = vertcat(gData.vSetKp.Views.Points{i,1}.Location, gData.vSetCkp.Views.Points{i,1}.Location);
% isValid = neighbourCheck(query, neighbors, minNeighbourDistance);
% InlierPoints1 = InlierPoints1(isValid, :);
% InlierPoints2 = InlierPoints2(isValid, :);
% Points3D = Points3D(isValid, :);

% % temp viewsets for bundleadjust & bundle adjust
tmp_v = imageviewset;
tmp_v = addView(tmp_v, 1, camPose1, "Points", InlierPoints1);
tmp_v = addView(tmp_v, 2, rigid3d(absOrien, absTrans), "Points", InlierPoints2);
tmp_v = addConnection(tmp_v, 1,2 , rigid3d(relOrien,relTrans) ,"Matches", [1:size(InlierPoints1,1); 1:size(InlierPoints1,1)]');



% Run bundleadjust -
tracks       = findTracks(tmp_v);
cameraPoses  = poses(tmp_v);
intrinsics = cameraIntrinsics(cameraParams.FocalLength,cameraParams.PrincipalPoint,size(I1));

[refinedPoints, refinedAbsPoses] = bundleAdjustment(Points3D, tracks, ...
    cameraPoses, intrinsics, 'FixedViewIDs', 1, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 20);

refinedAbsPose = refinedAbsPoses.AbsolutePose(2,1);
R = refinedAbsPose.Rotation;
t = refinedAbsPose .Translation;

% R = absOrien;
% t = absTrans;
% refinedPoints = Points3D;

% add pose data to global dataset
% kp
final_pts = vertcat(gData.vSetKp.Views.Points{i,1}.Location, InlierPoints2.Location) ;
connections = gData.vSetKp.Connections.Matches{i-1,1}; % this gonna be same
if Harris || FAST
    final_pts = cornerPoints(final_pts);
elseif ORB
    final_pts = ORBPoints(final_pts);
end
gData.vSetKp = updateView(gData.vSetKp, i, rigid3d(R,t), "Points", final_pts);
% gData.vSetKp = updateView(gData.vSetKp, i, "Points", final_pts);
gData.vSetKp = updateConnection(gData.vSetKp , i-1, i, "Matches", connections);


%% ckp
ckpoints2  = matchedPoints2(inliers == 0,:).Location;
% check if close to other points -
% isValid = neighbourCheck(ckpoints2, neighbors, minNeighbourDistance);
% ckpoints2 = ckpoints2(isValid, :);
% final ckp
fCkpoints2 = vertcat(gData.vSetCkp.Views.Points{i,1}.Location, ckpoints2 );

connections = gData.vSetCkp.Connections.Matches{i-1,1}; % this gonna be same
originalFeatures = gData.vSetCkp.Views.Features{i,1}; % this gonna be appended
addedFeatures = horzcat(R(:)'.*ones(size(ckpoints2,1),1), t.*ones(size(ckpoints2,1),1), ckpoints2);
featuresFinal = vertcat(originalFeatures, addedFeatures);

if Harris || FAST
    fCkpoints2 = cornerPoints(fCkpoints2);
elseif ORB
    fCkpoints2 = ORBPoints(fCkpoints2);
end

gData.vSetCkp = updateView(gData.vSetCkp, i, rigid3d(R, t), "Points", fCkpoints2 , "Features", featuresFinal);
% gData.vSetCkp = updateView(gData.vSetCkp, i, "Points", fCkpoints2 , "Features", featuresFinal);
gData.vSetCkp = updateConnection(gData.vSetCkp , i-1, i, "Matches", connections);


%% wp


[gData.wpSet, newPointIdx] = addWorldPoints(gData.wpSet, refinedPoints);
featureIndices = length(gData.vSetKp.Views.Points{i,1}.Location) - length(refinedPoints) + (1:length(InlierPoints2));
gData.wpSet = addCorrespondences(gData.wpSet, i, newPointIdx, featureIndices);



setPoints(KLT1, gData.vSetKp.Views.Points{i,1}.Location);
setPoints(KLTCand, gData.vSetCkp.Views.Points{i,1}.Location);

end