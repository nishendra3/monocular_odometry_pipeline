%% Setup
% Remove all question and TODO on all files before hand in
clear all;
close all;
clc;

ds = 2; % 0: KITTI, 1: Malaga, 2: parking
% Set your datasets folder accordingly
addpath(genpath('../datasets/'))
addpath(genpath('./funcs/'))
addpath('./plots/')

% Change ds on parameter initialization to be the same with the current
% file
control_script;
parameter_initialization;

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = '../datasets/kitti';
    assert(exist('kitti_path', 'var') ~= 0);
%     ground_truth = load([kitti_path '/poses/05.txt']);
%     ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];

    % Image Datastore
    imds = imageDatastore([kitti_path '/05/image_0']);
    I1 = readimage(imds,1);
    I2 = readimage(imds,3);
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path = '../datasets/malaga-urban-dataset-extract-07';
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];

    I1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    I2 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = '../datasets/parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);

    % Image Datastore
    imds = imageDatastore([parking_path '/images']);
    I1 = readimage(imds,1);
    I1 = rgb2gray(I1);
    I2 = readimage(imds,3);
    I2 = rgb2gray(I2);
else
    assert(false); 
end



% for Z = 0:50:499

% if mod(Z, 50) == 0   
    
   
    % gloabal data object
    gData.vSetKp = imageviewset;
    gData.vSetCkp = imageviewset;
    gData.wpSet = worldpointset;
    
   

    %% Bootstrap
    [KLT1, KLTCand, gData, ~] = BootstrapBA(I1, I2, K, gData);
% end

% if Z == 0
    % setup camera trajectory


    if plot_cam
        plotbins = setupPlot(gData);
    end

    % update Camera/Trajectory plot
    if plot_cam
        viewId = 2;
        updatePlot(viewId, gData, I2, plotbins);
    end

% end
%% Continuous operation
% range = (4:500);
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    
    if ds == 2
        image = readimage(imds, i);    
        image = rgb2gray(image);
    elseif ds == 0
        image = readimage(imds, i);
    else
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    end
    % Makes sure that plots refresh.    
    pause(0.01);
        
    prev_img = image;

%     [KLT1, KLTCand, gData] = continuous_operationBA(i-1, image, KLT1, KLTCand , K, gData);
%     [KLT1, KLTCand, gData] = continuous_operationBA(i-1-796, image, KLT1, KLTCand , K, gData);
    [KLT1, KLTCand, gData] = continuous_operationBA(i-bootstrap_frames(2)+2, image, KLT1, KLTCand , K, gData);

    % update plots
%     viewId = i-1;
%     viewId = i-1-796;
      viewId = i-bootstrap_frames(2)+2;

    if plot_cam
        updatePlot(viewId, gData, image, plotbins); 
    end
end

% end