function get_Intrinsics(separateFolder, cameraParameterFolder, squareSize, boardSize)
% =========================================================================
% Estimate camera intrinsics from processed images and save
% Input:
%   - separateFolder        : folder path containing processed images
%   - cameraParameterFolder : folder path to save estimated camera parameters
%   - squareSize            : size of one checkerboard square (in millimeters)
%   - boardSize             : checkerboard size (number of inner corners)
%
% Output:
%   - saves cameraParams.mat
% =========================================================================

    worldPoints = generateCheckerboardPoints(boardSize, squareSize);

    processedImageFiles = dir(fullfile(separateFolder, '*.png'));

    imagePoints = cell(1, numel(processedImageFiles));

    firstImage = imread(fullfile(processedImageFiles(1).folder, processedImageFiles(1).name));
    imageSize = size(firstImage, [1 2]);  

    for i = 1:numel(processedImageFiles)
        img = imread(fullfile(processedImageFiles(i).folder, processedImageFiles(i).name));
        [imagePoints{i}, boardSize] = detectCheckerboardPoints(img);
    end

    imagePointsForIntrinsics = cat(3, imagePoints{:});

    cameraParams = estimateCameraParameters(imagePointsForIntrinsics, worldPoints, ...
                                            'ImageSize', imageSize);

    if ~exist(cameraParameterFolder, 'dir')
        mkdir(cameraParameterFolder);
    end

    save(fullfile(cameraParameterFolder, 'cameraParams.mat'), 'cameraParams');
end
