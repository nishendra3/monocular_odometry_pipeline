function plotbins = setupPlot(gData)
close all;

figure();
set(gcf,'units','points','position',[200,200,1000,450]);

%Setup Trajectory Axis
ax1 = axes('Position',[.45 .1 .55 .8]);
grid minor
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
rotate3d on;
hold on
view(0, 0); 

% Get estimated camera location and orientation
viewId = 1;
trans = gData.vSetKp.Views.AbsolutePose(viewId,1).Translation; 
orient = gData.vSetKp.Views.AbsolutePose(viewId,1).Rotation;
direction = orient * [0, 0, 1]'; 
direction = 3*direction/norm(direction);
camerapose = direction + trans'; 

% Current Location: 
scatter3(trans(1), trans(2), trans(3), 40, 'red','filled', 'Marker','o');
hold on; 
% Current Direction: 
plot3([trans(1), camerapose(1)], [trans(2), camerapose(2)], [trans(3), camerapose(3)], 'red', 'LineWidth', 1.5); 

% Initialize camera trajectories
% Estimated Trajectory: 
etraj = plot3(0, 0, 0, 'b-', 'LineWidth',2);
% Total Landmarks: 
Totland = scatter3([], [], [],5,'black','filled','Marker','o');
% Current Landmarks: 
CurrLand = scatter3([], [], [],10,'red','filled','Marker','o');

legend([etraj, Totland, CurrLand], 'Estimated Trajectory',  ...
    'Total landmarks', 'Current Landmarks', 'Location','southeast');

% Setup Image Axes
ax2 = axes('Position',[.05 .10 .4 .8]);

plotbins.I = imshow([]); 

plotbins.axes1 = ax1; 
plotbins.axes2 = ax2;
end

