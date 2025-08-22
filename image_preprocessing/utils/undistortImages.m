function undistortImages(obj_orig_Folder, undistort_Folder, cameraParams)
% =========================================================================
% Undistort images in a folder and save results
% Input:
%   - obj_orig_Folder  : folder path containing original distorted images
%   - undistort_Folder : folder path to save undistorted images
%   - cameraParams     : camera parameter object (from calibration)
%
% Output:
%   - save undistorted images
% =========================================================================

    imageFiles = dir(fullfile(obj_orig_Folder, '*.*'));  

    if ~exist(undistort_Folder, 'dir')
        mkdir(undistort_Folder);
    end

    for i = 1:length(imageFiles)
        [~, ~, ext] = fileparts(imageFiles(i).name);
        
        if ismember(lower(ext), {'.bmp', '.jpg', '.jpeg', '.png', '.tiff', '.gif'})
            image = imread(fullfile(imageFiles(i).folder, imageFiles(i).name));
            undistortedImage = undistortImage(image, cameraParams);
            outputFileName = fullfile(undistort_Folder, imageFiles(i).name);
            imwrite(undistortedImage, outputFileName);
        end
    end
end