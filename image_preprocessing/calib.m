topdir = pwd;

utilsFolder = fullfile(topdir, 'utils');

addpath(genpath(topdir));

bgFolder = fullfile(topdir, 'calib', '2_bg');
maskFolder = fullfile(topdir, 'calib','3_mask');

threshold = 25;
minArea = 500;

maskNum = findMaskSort(bgFolder, threshold, minArea, maskFolder);

%% separate images
origFolder = fullfile(topdir, 'calib', '1_orig');
separateFolder = fullfile(topdir, 'calib', '4_separate');
orig_img = dir(fullfile(origFolder, '*.*')); 

for i = 1:length(orig_img)
    origImagePath = fullfile(origFolder, orig_img(i).name);
    separateWithMask(origImagePath, maskFolder, separateFolder, maskNum);
end
%% calibration
cameraParameterFolder = fullfile(topdir, 'calib', '5_cameraParameters');

imageFiles = dir(fullfile(separateFolder, '*.png'));

imageFileNames = cell(1, numel(imageFiles));

for i = 1:numel(imageFiles)
    imageFileNames{i} = fullfile(imageFiles(i).folder, imageFiles(i).name);
end

detector = vision.calibration.monocular.CheckerboardDetector();
[imagePoints, imagesUsed] = detectPatternPoints(detector, imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates for the planar pattern keypoints
squareSize = 1.270000e+00;  % in units of 'millimeters'
worldPoints = generateWorldPoints(detector, 'SquareSize', squareSize);

[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

%% visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'PatternCentric');
%h3=figure; showExtrinsics(cameraParams, 'CameraCentric');

displayErrors(estimationErrors, cameraParams);


%% save camera parameters
if ~exist(cameraParameterFolder, 'dir')
    mkdir(cameraParameterFolder);
end

save(fullfile(cameraParameterFolder, 'cameraParams.mat'), 'cameraParams');



%% undistort

numImages = numel(imageFileNames);
numPoints = size(imagePoints, 1);  
undistortedPoints = zeros(numPoints, 2, numImages);  % 30x2xnumImages
imagePoints_after_un = zeros(numPoints, 2, numImages);

outputFolderRaw = fullfile(topdir, 'calib', '6_raw');
if ~exist(outputFolderRaw, 'dir')
    mkdir(outputFolderRaw);
end

for i = 1:numImages
    imOrig = imread(imageFileNames{i});
    
    imWithMarkers = insertMarker(imOrig, imagePoints(:, :, i), 'Color', 'red', 'Size', 5);
    
    [~, originalName, ext] = fileparts(imageFileNames{i}); 
    outputFileName = fullfile(outputFolderRaw, [originalName, ext]);
    imwrite(imWithMarkers, outputFileName);
end


outputFolderUndistorted = fullfile(topdir, 'calib', '7_undistorted');
if ~exist(outputFolderUndistorted, 'dir')
    mkdir(outputFolderUndistorted);
end



for i = 1:numImages

    imOrig = imread(imageFileNames{i});

    undistortedPoints(:, :, i) = undistortPoints(imagePoints(:, :, i), cameraParams.Intrinsics);


    [im, newIntrinsics] = undistortImage(imOrig, cameraParams.Intrinsics, 'OutputView', 'full');


    newOrigin = cameraParams.Intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
    imagePoints_after_un(:, :, i) = undistortedPoints(:, :, i) - newOrigin; 


    imWithMarkers = insertMarker(im, imagePoints_after_un(:, :, i), 'Color', 'red', 'Size', 5);


    [~, originalName, ext] = fileparts(imageFileNames{i}); 
    outputFileName = fullfile(outputFolderUndistorted, [originalName, ext]);
    imwrite(imWithMarkers, outputFileName);
end


%%  extrinsic
numCameras = 8;  
camExtrinsics = rigidtform3d.empty(numCameras, 0);
for i = 1:numCameras
    camExtrinsics(i)= estimateExtrinsics(imagePoints(:,:,i), cameraParams.WorldPoints, cameraParams.Intrinsics);
end




extrinsicsArray_mat = rigidtform3d.empty(numCameras, 0);
% Process the first eight images
for i = 1:numCameras
    extrinsicsArray_mat(i) = estimateExtrinsics(imagePoints_after_un(:,:,i), cameraParams.WorldPoints, newIntrinsics);
end




%% show the first camera pose

figure;
hold on;
axis([-200 200 -200 200 -200 0])
pcshow([worldPoints,zeros(size(cameraParams.WorldPoints,1),1)], ...
            VerticalAxisDir="down",MarkerSize=40);
for i = 1:numel(extrinsicsArray_mat)
    camPose = extr2pose(extrinsicsArray_mat(i));
    plotCamera(AbsolutePose=camPose,Size=5, Label=['Cam' num2str(i)], Color= 'red');
end

for i = 1:numel(camExtrinsics)
    camPose = extr2pose(camExtrinsics(i));
    plotCamera(AbsolutePose=camPose,Size=5, Label=['Cam' num2str(i)], Color= 'blue');
end
hold off;



%%

numCameras = 8;  
numBoards = 15;  
pointsPerBoard = 30;  

% Generate the 3D coordinates of the first calibration board (n x 3 matrix)
baseWorldPoints = cameraParams.WorldPoints;  % n x 2
xyzPoints = [baseWorldPoints, zeros(size(baseWorldPoints, 1), 1)];  % n x 3


[rotationMatrices, translationVectors] = getRotationAndTranslation(cameraParams);

% Using the first calibration board as the reference
R1 = rotationMatrices(:, :, 1);  
T1 = translationVectors(1, :)';  

allXYZPoints = xyzPoints;  

% Only iterate over the 1st, 9th, 17th, ... calibration boards
for boardIdx = 1:8:numBoards*numCameras
    if boardIdx == 1
        continue; 
    end


    R_curr = rotationMatrices(:, :, boardIdx);
    T_curr = translationVectors(boardIdx, :)';

    currentWorldPoints = [baseWorldPoints, zeros(size(baseWorldPoints, 1), 1)];  % n x 3

    % Convert the corner points of the current calibration board to the camera coordinate system
    pointsInCamera = R_curr * currentWorldPoints' + T_curr;  % 3 x n

    % to the coordinate system of the first calibration board.
    pointsInFirstBoard = R1 \ (pointsInCamera - T1);  % 3 x n

    allXYZPoints = [allXYZPoints; pointsInFirstBoard'];
end
%% visualize

figure;
plot3(allXYZPoints(:, 1), allXYZPoints(:, 2), allXYZPoints(:, 3), 'bo');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Points of All Calibration Boards');
grid on;
axis equal;

%%


% pointTracks  1 x (numBoards * pointsPerBoard)
data_extrinsic.pointTracks(1, numBoards * pointsPerBoard) = pointTrack();

% 
for pointIdx = 1:(numBoards * pointsPerBoard)

    points = zeros(numCameras, 2);

    % get the 2D image coordinates
    for camIdx = 1:numCameras
        boardIdx = floor((pointIdx - 1) / pointsPerBoard) + 1; 
        pointLocalIdx = mod((pointIdx - 1), pointsPerBoard) + 1; 
        points(camIdx, :) = imagePoints_after_un(pointLocalIdx, :, (boardIdx - 1) * numCameras + camIdx);
    end
    
    data_extrinsic.pointTracks(1, pointIdx) = pointTrack(uint32((1:numCameras)'), points);
end


%% bundleAdjustment

intrinsics = cameraParams.Intrinsics;  
for i = 1:numel(extrinsicsArray_mat)
    AbsolutePose(i) = extr2pose(extrinsicsArray_mat(i));
end

data_extrinsic.cameraPoses = table(uint32((1:numCameras)'), AbsolutePose', 'VariableNames', {'ViewId', 'AbsolutePose'});
data_extrinsic.xyzPoints = allXYZPoints;       
data_extrinsic.intrinsics = intrinsics; 


imagePoint = worldToCamera2D(allXYZPoints(5,:)',extrinsicsArray_mat(1, 1).A, newIntrinsics.K);
(imagePoint(1,1)-data_extrinsic.pointTracks(1, 5).Points(1,1))^2 + (imagePoint(1,2)-data_extrinsic.pointTracks(1, 5).Points(1,2))^2;


[xyzRefinedPoints,refinedPoses, reprojectionError] = ...
    bundleAdjustment(data_extrinsic.xyzPoints,data_extrinsic.pointTracks,data_extrinsic.cameraPoses,data_extrinsic.intrinsics);
%% show refinedPoses
 
pcshowpair(pointCloud(data_extrinsic.xyzPoints), pointCloud(xyzRefinedPoints), ...
    AxesVisibility="on", VerticalAxis="y", VerticalAxisDir="down", MarkerSize=40);
hold on
plotCamera(data_extrinsic.cameraPoses, Size=5, Color="m");
plotCamera(refinedPoses, Size=5, Color="g");
legend("Before refinement", "After refinement", color="w");

%%
meanError = mean(reprojectionError, 'omitnan');

fprintf('Mean reprojection error: %.2f pixels\n', meanError);

figure;
histogram(reprojectionError, 50); 
title('Reprojection Error Histogram');
xlabel('Reprojection Error (pixels)');
ylabel('Frequency');
grid on;


%% save

load(fullfile(cameraParameterFolder, 'cameraParams.mat'));

for i = 1 : numCameras
    refinedExtrinsics(i) = pose2extr(refinedPoses.AbsolutePose(i,1));
end

save(fullfile(cameraParameterFolder, 'refinedExtrinsics.mat'), 'refinedExtrinsics');

%%
figure;
hold on;
axis([-200 200 -200 200 -200 0])
pcshow([worldPoints,zeros(size(cameraParams.WorldPoints,1),1)], ...
            VerticalAxisDir="down",MarkerSize=40);
for i = 1:numel(refinedExtrinsics)
    camPose = extr2pose(refinedExtrinsics(i));
    plotCamera(AbsolutePose=camPose,Size=5, Label=['Cam' num2str(i)], Color= 'red');
end

hold off;










