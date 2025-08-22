function pixel_value = bilinear(image, u_orig, v_orig)
    % Get the dimensions of the input image
    [height, width, ~] = size(image);
    
    % Get the four neighboring pixel coordinates
    x1 = floor(u_orig); x2 = ceil(u_orig);
    y1 = floor(v_orig); y2 = ceil(v_orig);
    
    % Ensure the coordinates are within image bounds
    if x1 < 1 || x2 > width || y1 < 1 || y2 > height
        pixel_value = 0;  % Return 0 if out of bounds
        return;
    end
    
    % Calculate interpolation weights
    wx = u_orig - x1;  % Weight for x-direction
    wy = v_orig - y1;  % Weight for y-direction

    % Get the 4 surrounding pixel values (convert to double for interpolation)
    I11 = double(image(y1, x1, :));  % Top-left
    I12 = double(image(y1, x2, :));  % Top-right
    I21 = double(image(y2, x1, :));  % Bottom-left
    I22 = double(image(y2, x2, :));  % Bottom-right

    % Perform bilinear interpolation
    pixel_value = (1 - wx) * (1 - wy) * I11 + ...
                  wx * (1 - wy) * I12 + ...
                  (1 - wx) * wy * I21 + ...
                  wx * wy * I22;
    
    % Convert the result back to the original image type (e.g., uint8)
    pixel_value = cast(pixel_value, class(image));
end