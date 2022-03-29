% Choose the algorithm that shall run in every case

%% Feature Detectors (Harris and ORB were tuned)
Harris = true;              % return cornerPoints object
ORB = false;                % returns an ORBPoints object
FAST = false;               % return cornerPoints object
SURF = false;               % return SURFPoints object
% else SIFT                 % return SIFTPoints object

%% For Feature tracking
KLT_check = true;   % else extractFeatures

%% Ploting
plot_cam = true;