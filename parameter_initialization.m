ds = 2;
control_script;
if ds == 0
    %% Feature Detectors
    
    % Harris detector 
    harris_min_quality = 0.01;
    harris_filter_size = 5; 
    
    % Fast detector
    fast_min_quality = 0.01;
    fast_min_contrast = 0.2; 
    
    %ORB detector
    ORB_scale_factor = 1.2;
    ORB_num_levels = 8;
    
    %% KLT parameters
    klt_NumPyramidLevels = 4;                          
    klt_MaxBidirectionalError = 0.3;                          
    klt_BlockSize = [31 31];
    klt_MaxIterations = 100;   
    
    %% Frames
    bootstrap_frames = [1, 3];
%     bootstrap_frames = [797, 799];
    % last_frame = 1500; % Different for every dataset normally
    
    %% Triangulation
    % points3d_threshold = 100;
    landmark_threshold = 100; % 1500 orb
    
    %% Retriangulation - converts ckp to kp if landmarks reduce
    rt_alpha = deg2rad(0.4);
    rt_alpha_max = deg2rad(8);
%     minretrerror = 0.1;
    
    %% ReBootstrap - detects new features if we are not tracking enough
    minNeighbourDistance = 20;
    ckp_threshold = 150; % 1000 orb

    %% BA
    BAinterval = 1;
    windowSize = 20;
elseif ds == 1
    %% Feature Detectors
    
    % Harris detector 
    harris_min_quality = 0.01;
    harris_filter_size = 5; 
    
    % Fast detector
    fast_min_quality = 0.01;
    fast_min_contrast = 0.2; 
    
    %ORB detector
    ORB_scale_factor = 1.2;
    ORB_num_levels = 8;
    
    %% KLT parameters
    klt_NumPyramidLevels = 4;                          
    klt_MaxBidirectionalError = 0.3;                          
    klt_BlockSize = [31 31];
    klt_MaxIterations = 100;   
    
    %% Frames
%     bootstrap_frames = [1, 3];
    bootstrap_frames = [630, 632];
    % last_frame = 1500; % Different for every dataset normally
    
    %% Triangulation
    % points3d_threshold = 100;
    landmark_threshold = 120; % 1500 orb
    
    %% Retriangulation - converts ckp to kp if landmarks reduce
    % rt_alpha = deg2rad(3);
    rt_alpha = deg2rad(2);
    rt_alpha_max = deg2rad(30);
%     minretrerror = 0.1;
    
    %% ReBootstrap - detects new features if we are not tracking enough
    minNeighbourDistance = 20;
    ckp_threshold = 140; % 1000 orb
    
    %% BA
    BAinterval = 2;
    windowSize = 10;
elseif ds == 2
    %% Feature Detectors
    
    % Harris detector 
    harris_min_quality = 0.0001;
    harris_filter_size = 5; 
    
    % Fast detector
    fast_min_quality = 0.01;
    fast_min_contrast = 0.2; 
    
    %ORB detector
    ORB_scale_factor = 1.2;
    ORB_num_levels = 8;
    
    %% KLT parameters
    klt_NumPyramidLevels = 4;                          
    klt_MaxBidirectionalError = 0.3;                          
    klt_BlockSize = [31 31];
    klt_MaxIterations = 100;   
    
    %% Frames
    bootstrap_frames = [1, 4];
%     bootstrap_frames = [797, 799];
    % last_frame = 1500; % Different for every dataset normally
    
    %% Triangulation
    % points3d_threshold = 100;
    landmark_threshold = 150; % 250 for orb
    if ORB
        landmark_threshold = 250;
    end

    %% Retriangulation - converts ckp to kp if landmarks reduce
    rt_alpha = deg2rad(2);
    rt_alpha_max = deg2rad(40);
%     minretrerror = 0.1;
    
    %% ReBootstrap - detects new features if we are not tracking enough
    minNeighbourDistance = 20;
    ckp_threshold = 150; % 1000 orb

    %% BA
    BAinterval = 1;
    windowSize = 10;
end
