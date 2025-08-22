function maskNum = findMaskSort(bgFolder, threshold, minArea, maskFolder)
% =========================================================================
% Find background masks and save
% Input:
%   - bgFolder   : folder path containing background images (.bmp)
%   - threshold  : intensity threshold for binarization
%   - minArea    : minimum region area (pixels) to keep
%   - maskFolder : folder path to save the generated mask images
%
% Output:
%   - maskNum    : number of masks found and saved
% =========================================================================

    bgFiles = dir(fullfile(bgFolder, '*.bmp'));
    bgImagePath = fullfile(bgFolder, bgFiles(1).name);
    bg = imread(bgImagePath);

    if size(bg, 3) == 3
        bg = rgb2gray(bg);
    end

    binaryImage = bg > threshold;

    binaryImage = imfill(binaryImage, 'holes');
    binaryImage = bwareaopen(binaryImage, minArea); 

    regions = regionprops(binaryImage, 'BoundingBox', 'Area', 'PixelIdxList', 'Centroid');

    imgHeight = size(bg, 1);
    imgWidth = size(bg, 2);
    centerX = imgWidth / 2;
    centerY = imgHeight / 2;

    angles = zeros(length(regions), 1);
    for j = 1:length(regions)
        centroid = regions(j).Centroid;
        dx = centroid(1) - centerX;
        dy = centerY - centroid(2);  
        angle = atan2(dy, dx);   
        angles(j) = mod(angle - pi/2, 2*pi);  
    end

    [~, sortedIndices] = sort(angles);

    if ~exist(maskFolder, 'dir')
        mkdir(maskFolder);
    end

    masks = cell(1, length(regions));

    for j = 1:length(regions)

        regionIdx = sortedIndices(j);
        masks{j} = false(size(binaryImage));  
        masks{j}(regions(regionIdx).PixelIdxList) = true; 

        imwrite(masks{j}, fullfile(maskFolder, ['frame000000_cam' num2str(j-1,'%03d') '.png']));
    end

    % Return number of masks
    maskNum = length(regions);
end
