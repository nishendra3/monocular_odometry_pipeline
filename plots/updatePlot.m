function updatePlot(viewId, gData, I, plotbins)
warning off;  

% Plot Location and Orientation
  
trans = gData.vSetKp.Views.AbsolutePose(viewId,1).Translation; 
orient = gData.vSetKp.Views.AbsolutePose(viewId,1).Rotation;
direction = orient' * [0, 0, 1]'; 
direction = 3*direction/norm(direction); 
camerapose = direction + trans'; 

h = plotbins.axes1.Children(5);
set(h, 'XData', trans(1), 'YData', trans(2), 'ZData', trans(3));
h = plotbins.axes1.Children(4);
set(h, 'XData', [trans(1), camerapose(1)], 'YData',[trans(2), camerapose(2)], 'ZData', [trans(3), camerapose(3)]);

% Plot the estimated trajectory
h = plotbins.axes1.Children(3);
locations = cat(1, gData.vSetKp.Views.AbsolutePose(:,1).Translation);
set(h, 'XData', locations(:,1), 'YData', ...
    locations(:,2), 'ZData', locations(:,3));

% Updata point cloud
h = plotbins.axes1.Children(2);
global_landmarks = gData.wpSet.WorldPoints; 
set(h, 'XData', global_landmarks(:,1), 'YData', ...
    global_landmarks(:,2), 'ZData', global_landmarks(:,3));

% Updata inlier point cloud
h = plotbins.axes1.Children(1);
pointIndices = findWorldPointsInView(gData.wpSet, viewId);
curr_landmarks = global_landmarks(pointIndices,:); 
set(h, 'XData', curr_landmarks(:,1), 'YData', ...
    curr_landmarks(:,2), 'ZData', curr_landmarks(:,3));

% set axis limits 5 meters larger than data
limitx=get(plotbins.axes2,'XLim');
set(plotbins.axes1,'Xlim',[trans(1)-50, trans(1)+50]); 

limity=get(plotbins.axes2,'YLim');
set(plotbins.axes1,'Ylim',[trans(2)-50, trans(2)+50]); 

limitz=get(plotbins.axes2,'ZLim');
set(plotbins.axes1,'Zlim',[trans(3)-50, trans(3)+50]); 

% Second plot
axes(plotbins.axes2);   
imshow(I); 
num_landmarks = length(curr_landmarks); 
% title(['Landmarks: ',num2str(num_landmarks)], 'Position','south');
xlabel(['Landmarks: ',num2str(num_landmarks)]);
hold on; 

%draw points
scatter(gData.vSetCkp.Views.Points{viewId,1}.Location(:,1), gData.vSetCkp.Views.Points{viewId,1}.Location(:,2), 5, 'red', 'filled', 'Marker', 'o'); 
scatter(gData.vSetKp.Views.Points{viewId,1}.Location(:,1), gData.vSetKp.Views.Points{viewId,1}.Location(:,2), 5, 'green', 'Marker', '+'); 

hold off; 
legend('candidate keypoints', 'keypoints');

end

