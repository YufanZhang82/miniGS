function imagePoint = worldToCamera2D(worldPoint, camExtrinsics, intrinsics)
% =========================================================================
% Project 3D world coordinates to 2D camera image coordinates
% Input:
%   - worldPoint   : 3x1 vector, 3D world coordinate
%   - camExtrinsics: 4x4 camera extrinsic matrix
%   - intrinsics   : 3x3 camera intrinsic matrix
%
% Output:
%   - imagePoint   : 1x2 vector, 2D image coordinate in pixels
% =========================================================================

    if size(worldPoint, 1) ~= 3 || size(worldPoint, 2) ~= 1
        error('worldPoint must be a 3x1 vector');
    end

    worldPointHomogeneous = [worldPoint; 1]; 

    cameraPoint = camExtrinsics * worldPointHomogeneous; 

    if cameraPoint(3) == 0
        error('Z coordinate in camera point is zero, cannot project to image.');
    end
    cameraPointNormalized = cameraPoint / cameraPoint(3); 

    imagePointHomogeneous = intrinsics * cameraPointNormalized(1:3); 
    
    imagePoint = imagePointHomogeneous(1:2)'; 
end
