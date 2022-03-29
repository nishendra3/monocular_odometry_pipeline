function [KLT1, KLTCand, gData] = continuous_operationBA(i, image, KLT1, KLTCand , K, gData)

control_script;
parameter_initialization;

[key_points, kp_validity] = step(KLT1, image);
kp_validity(key_points(:,1) < 0 | key_points(:,2) < 0) = 0;
kp_matched = key_points(kp_validity,:);

%convert to cornerPoint object to match template 
if Harris || FAST   
    kp_matched = cornerPoints(kp_matched);
elseif ORB
    kp_matched = ORBPoints(kp_matched);
end



% update 3D point correspondances
%gData.wpSet = addWorldPoints(gData.wpSet, Points3D);
indices = findWorldPointsInView(gData.wpSet, i-1);
Points3D = gData.wpSet.WorldPoints(indices,:);
pointIndices = indices(kp_validity);
featureIndices = 1:size(kp_matched);
gData.wpSet = addCorrespondences(gData.wpSet, i, pointIndices, featureIndices);


% Calculate Pose using P3P Ransac
cameraParams = cameraParameters('IntrinsicMatrix', K');
[R, t] = estimateWorldCameraPose(kp_matched.Location, Points3D(kp_validity,:), cameraParams);


% Match the keypoints from previous frame to current frame
row1 = 1:size(key_points,1);
row2 = 1:size(kp_matched,1);
indexPair = [ row1(kp_validity); row2 ]'; 
gData.vSetKp = addView(gData.vSetKp, i, rigid3d(R, t), "Points" , kp_matched);
gData.vSetKp = addConnection(gData.vSetKp, i-1, i, "Matches", indexPair ); %TODO: verify format of indexPair

% Compute Candidate keypoints
[candidate_key_points, ckp_validity] = step(KLTCand,image);
%convert to appropriate object
ckp_validity(candidate_key_points(:,1) < 0 | candidate_key_points(:,2) < 0) = 0;
ckp_tracked = candidate_key_points(ckp_validity,:);

if Harris || FAST   
    ckp_tracked = cornerPoints(ckp_tracked);
elseif ORB
    ckp_tracked = ORBPoints(ckp_tracked);
end

% Match the keypoints from previous frame to current frame
row1 = 1:size(candidate_key_points,1);
row2 = 1:size(ckp_tracked,1);
indexPairCKP = [ row1(ckp_validity); row2 ]'; 
oldTracking = gData.vSetCkp.Views.Features{i-1,1};
oldTrackingLeft = oldTracking(ckp_validity,:);
gData.vSetCkp = addView(gData.vSetCkp, i, rigid3d(R, t), "Points" , ckp_tracked, "Features", oldTrackingLeft);
gData.vSetCkp = addConnection(gData.vSetCkp, i-1, i, "Matches", indexPairCKP ); %TODO: verify format of indexPair


%% Retriangulate
currentLandmarks = findWorldPointsInView(gData.wpSet, i);
numCL = length(currentLandmarks);


if numCL < landmark_threshold
    % find bearing angle
    Ri = reshape(oldTrackingLeft(:,1:9)',3,3,[]);
    Ti = reshape(oldTrackingLeft(:,10:12)',1,3,[]);
    Ui = oldTrackingLeft(:,13:14);
    Rf = R;
    Uf = ckp_tracked.Location;

    alpha = calcAlpha(Ri,Ui,Rf,Uf,K);
    alpha_valid = alpha > rt_alpha & alpha < rt_alpha_max;
    % find alpha
    cameraParams = cameraParameters('IntrinsicMatrix',K');
    Ri =  Ri(:,:,alpha_valid);
    Ti =  Ti(:,:,alpha_valid);
    Pi =  Ui(alpha_valid,:);

    [Rf, Tf] =  cameraPoseToExtrinsics(R, t);
    M2 = cameraMatrix(cameraParams, Rf, Tf);
    Pf =  Uf(alpha_valid,:);

    range = 1:size(Pf,1);
    new3Dpts = [];
    new_kps = [];

    % retriangulate
    for pt = range
        % Find rot, trans for initial point
        Rfi = Ri(:,:,pt);
        Tfi = Ti(:,:,pt);
        % Find camera matrix for initial point
        [Rfi, Tfi] =  cameraPoseToExtrinsics(Rfi, Tfi);
        M1 = cameraMatrix(cameraParams, Rfi, Tfi);
        % Take the initial point and corresponding current point
        Pi1 = Pi(pt,:);
        Pf1 = Pf(pt,:);

        % Triangualte them to find new point 
        [newPoint3D, ~, isValid] = triangulate(Pi1, Pf1, M1 ,M2);
        newPoint3D = newPoint3D(isValid,:);
%         if reperror > minretrerror
%             newPoint3D = [];
%         end
        if ~isempty(newPoint3D)
            new3Dpts(end+1,:) = newPoint3D;
            new_kps(end+1,:) = Pf1;
        end
    end

    % for ckp's where a<threshold, we still track them
    if ~isempty(new3Dpts)
        % add new 3D points and assign correspondences to kp's
        len_kp = size(kp_matched,1);
        idx_new_kps = (1 : size(new_kps,1)) + len_kp;

        [gData.wpSet, newPointIndices] = addWorldPoints(gData.wpSet, new3Dpts);
        gData.wpSet = addCorrespondences(gData.wpSet, i, newPointIndices,idx_new_kps);

        % convert new_kps to harris/orb points
        if Harris || FAST   
            new_kps = cornerPoints(new_kps);
        elseif ORB
            new_kps = ORBPoints(new_kps);
        end

        % update kp dataset
        final_kps = vertcat(kp_matched, new_kps);
        gData.vSetKp = updateView(gData.vSetKp, i, "Points" , final_kps);
        gData.vSetKp = updateConnection(gData.vSetKp, i-1, i, "Matches" , indexPair);

        % update ckp dataset
        ckp_left = ckp_tracked(alpha_valid==0,:);
        features_left = oldTrackingLeft(alpha_valid==0,:);
        gData.vSetCkp = updateView(gData.vSetCkp, i, "Points" , ckp_left, "Features", features_left);
        row1 = 1:size(candidate_key_points,1);
        row1 = row1(ckp_validity);
        row2 = 1:size(ckp_left,1);
        % Match the keypoints from previous frame to current frame
        indexPairCkp_left = [ row1(alpha_valid==0); row2 ]';
        gData.vSetCkp = updateConnection(gData.vSetCkp, i-1, i, "Matches" , indexPairCkp_left);

    end
end

%% Create pointcloud for new keyframe
if length(gData.vSetCkp.Views.Points{i,1}.Location) < ckp_threshold
        
    [~,~,~,query] = BootstrapBA(image);
    neighbors = vertcat(gData.vSetKp.Views.Points{i,1}.Location, gData.vSetCkp.Views.Points{i,1}.Location);
    isValid = neighbourCheck(query, neighbors, minNeighbourDistance);
    newCKpoints = query(isValid, :);

    % update CKpoints in global data
    CKfinal = vertcat(gData.vSetCkp.Views.Points{i,1}.Location, newCKpoints);
    connections = gData.vSetCkp.Connections.Matches{i-1,1}; % this gonna be same
    originalFeatures = gData.vSetCkp.Views.Features{i,1}; % this gonna be appended
    addedFeatures = horzcat(R(:)'.*ones(size(newCKpoints,1),1), t.*ones(size(newCKpoints,1),1), newCKpoints);
    featuresFinal = vertcat(originalFeatures, addedFeatures);
    
    % convert points
    if Harris || FAST   
        CKfinal = cornerPoints(CKfinal);
    elseif ORB
        CKfinal = ORBPoints(CKfinal);
    end
    gData.vSetCkp = updateView(gData.vSetCkp, i, "Points", CKfinal, "Features", featuresFinal);
    gData.vSetCkp = updateConnection(gData.vSetCkp , i-1, i, "Matches", connections);
end



viewId = i;
if mod(viewId, BAinterval) == 0        
    % Find point tracks in the last WindowSize views and triangulate.
    startFrame = max(1, viewId - windowSize);
    tracks = findTracks(gData.vSetKp, startFrame:viewId);
    camPoses = poses(gData.vSetKp, startFrame:viewId);
    intrinsics = cameraIntrinsics(cameraParams.FocalLength,cameraParams.PrincipalPoint,size(image));
    
    pointIndices = findWorldPointsInTracks(gData.wpSet,tracks);
    xyzPoints = gData.wpSet.WorldPoints(pointIndices, :);
    
                            
    % Hold the first two poses fixed, to keep the same scale. 
    fixedIds = [startFrame, startFrame+1];
    
    [refinedPoints, camPoses] = bundleAdjustment(xyzPoints, tracks, ...
        camPoses, intrinsics, 'FixedViewIDs', fixedIds, ...
        'PointsUndistorted', true, 'AbsoluteTolerance', 1e-12,...
        'RelativeTolerance', 1e-12, 'MaxIterations', 200);
    
    gData.vSetKp = updateView(gData.vSetKp, camPoses); % Update view set.
    gData.wpSet  = updateWorldPoints(gData.wpSet, pointIndices, refinedPoints);

end

setPoints(KLT1, gData.vSetKp.Views.Points{i,1}.Location);
setPoints(KLTCand, gData.vSetCkp.Views.Points{i,1}.Location);


end