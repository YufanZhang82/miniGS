function separateWithMask(imageFolder, maskFolder, saveFolder, maskNum)
% =========================================================================
% Apply masks to images and save the separated results
% Input:
%   - imageFolder : folder path containing images to be processed (.bmp/.png)
%   - maskFolder  : folder path containing mask images
%   - saveFolder  : folder path to save the masked output images
%   - maskNum     : number of masks
%
% Output:
%   save separated images 
% =========================================================================

        imageFiles = [dir(fullfile(imageFolder, '*.bmp')); ...
                  dir(fullfile(imageFolder, '*.png'))];

    if ~exist(saveFolder, 'dir')
        mkdir(saveFolder);
    end

    for i = 1:length(imageFiles)
        image = imread(fullfile(imageFiles(i).folder, imageFiles(i).name));

        for j = 1:maskNum
            mask = imread(fullfile(maskFolder, ['frame000000_cam' num2str(j-1, '%03d') '.png']));
            croppedImage = bsxfun(@times, image, cast(mask, 'like', image));
            outputFileName = fullfile(saveFolder, ['frame' sprintf('%06d', i-1) '_cam' num2str(j-1, '%03d') '.png']);
            imwrite(croppedImage, outputFileName, 'png');
        end
    end
end
