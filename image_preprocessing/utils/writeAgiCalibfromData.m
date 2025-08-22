function writeAgiCalibfromData(intrinsic, extrinsic, dist, camRes, data_dir, save_filename, frame_id)

    numCam = size(intrinsic,3);
    
    K = {};
    D = {};
    R = {};
    T = {};
    
    intri_temp = zeros(3);
    dist_temp = zeros(5,1);
    rot_temp = eye(3);
    trans_temp = zeros(3,1);
    
    property_name = 'fixed';
    property_value= 'false';
    calibration_type = 'frame';
    calibration_class = 'adjusted';
    
%% camera resolution

    cam_resolution = {};
    for ithCam = 1:numCam
        cam_resolution{ithCam} = [camRes(1), camRes(2)]; % w, h
    end
    
%% load calibration
    
    for ithCam = 1:numCam
        
        intri_temp(:,:) = intrinsic(:,:,ithCam);
        K{ithCam} = intri_temp(:,:);
        
        dist_temp(:) = dist(1,:,ithCam);
        D{ithCam} = dist_temp;
        
        rot_temp(:,:) = extrinsic(1:3,1:3,ithCam);
        trans_temp(:) = extrinsic(1:3,4,ithCam);
        
        R{ithCam} = rot_temp;
        T{ithCam} = trans_temp;
    
        T{ithCam} = -R{ithCam}' * T{ithCam};
        R{ithCam} = R{ithCam}';
        
    end
        
 %% obtain camera pos, lookat, up, right
 
     for ithCam = 1:numCam

        camera_trans_set{ithCam} = [R{ithCam}, T{ithCam}; 0, 0, 0, 1];
        camera_trans_set{ithCam} = camera_trans_set{ithCam}';
        camera_trans_set{ithCam} = camera_trans_set{ithCam}(:)';
     end
     
 %% init center

    center_set = [0.0, 0.0, 845.0];
    size_set = ones(1, 3)*300.0;
    iden = eye(3,3);
    R_set = iden(:)';
    
%% write agi script
docNode = com.mathworks.xml.XMLUtils.createDocument...
    ('document');
docRootNode = docNode.getDocumentElement;
docRootNode.setAttribute('version','1.2.0');
chunk = docNode.createElement('chunk');
%chunk.setAttribute('target','upslope_product_page.html');
% chunk.appendChild(docNode.createTextNode('Upslope Area Toolbox'));
ChunkNode = docRootNode.appendChild(chunk);
sensors = docNode.createElement('sensors');
SensorsNode = ChunkNode.appendChild(sensors);

% intrinsic
for i = 1:numCam
    sensor = docNode.createElement('sensor');
    %   sensor.setAttribute('id', sprintf('%i',i), 'label', sprintf('%03d_%03d.bmp',100,100), 'type', 'frame' );
    row_id = ceil(i/36);
    col_id = rem(i,36);
    if col_id == 0
        col_id = 36;
    end
    sensor.setAttribute('id', sprintf('%i', i));
    sensor.setAttribute('label', ['frame' num2str(frame_id,'%06d') '_cam' num2str(i-1,'%03d')]);
    sensor.setAttribute('type', 'frame');
    SensorNode = SensorsNode.appendChild(sensor);
    
    resolution = docNode.createElement('resolution');
    %    resolution.setAttribute('width', num2str(m_width), 'height', num2str(m_height) );
    resolution.setAttribute('width', num2str(cam_resolution{i}(1)));
    resolution.setAttribute('height', num2str(cam_resolution{i}(2)));
    ResolutionNode = SensorNode.appendChild(resolution);
    
    property = docNode.createElement('property');
    %    property.setAttribute('name',property_name , 'value', property_value );
    property.setAttribute('name', property_name);
    property.setAttribute( 'value', property_value );
    PropertyNode = SensorNode.appendChild(property);
    
    calibration = docNode.createElement('calibration');
    %    calibration.setAttribute('type',property_name , 'class', property_value );
    calibration.setAttribute('type', calibration_type);
    calibration.setAttribute( 'class', calibration_class );
    CalibrationNode = SensorNode.appendChild(calibration);
    
    resolution = docNode.createElement('resolution');
    resolution.setAttribute('width', num2str(cam_resolution{i}(1)));
    resolution.setAttribute('height', num2str(cam_resolution{i}(2)));
    ResolutionNode = CalibrationNode.appendChild(resolution);
    
    fx = docNode.createElement('fx');
    fx.appendChild(docNode.createTextNode(sprintf('%f', K{i}(1, 1))));
    ResolutionNode = CalibrationNode.appendChild(fx);
    
    fy = docNode.createElement('fy');
    fy.appendChild(docNode.createTextNode(sprintf('%f', K{i}(2, 2))));
    ResolutionNode = CalibrationNode.appendChild(fy);
    
    cx = docNode.createElement('cx');
    cx.appendChild(docNode.createTextNode(sprintf('%f', K{i}(1, 3))));
    ResolutionNode = CalibrationNode.appendChild(cx);
    
    cy = docNode.createElement('cy');
    cy.appendChild(docNode.createTextNode(sprintf('%f', K{i}(2, 3))));
    ResolutionNode = CalibrationNode.appendChild(cy);
    
    k1 = docNode.createElement('k1');
    k1.appendChild(docNode.createTextNode(sprintf('%f', D{i}(1))));
    ResolutionNode = CalibrationNode.appendChild(k1);
    
    k2 = docNode.createElement('k2');
    k2.appendChild(docNode.createTextNode(sprintf('%f', D{i}(2))));
    ResolutionNode = CalibrationNode.appendChild(k2);
    
    k3 = docNode.createElement('k3');
    k3.appendChild(docNode.createTextNode(sprintf('%f', D{i}(5))));
    ResolutionNode = CalibrationNode.appendChild(k3);
    
    p1 = docNode.createElement('p1');
    p1.appendChild(docNode.createTextNode(sprintf('%f', D{i}(3))));
    ResolutionNode = CalibrationNode.appendChild(p1);
    
    p2 = docNode.createElement('p2');
    p2.appendChild(docNode.createTextNode(sprintf('%f', D{i}(4))));
    ResolutionNode = CalibrationNode.appendChild(p2);
end

% extrinsic
cameras = docNode.createElement('cameras');
CamerasNode = ChunkNode.appendChild(cameras);
for i = 1:numCam
    camera = docNode.createElement('camera');
    %sensor_id_v = sprintf('%i',i);
    row_id = ceil(i/36);
    col_id = rem(i,36);
    if col_id == 0
        col_id = 36;
    end

    %   sensor.setAttribute('id', sprintf('%i',i), 'label', sprintf('%03d_%03d.bmp',100,100), 'type', 'frame' );
    camera.setAttribute('id', sprintf('%i', i));
    camera.setAttribute('label', ['frame' num2str(frame_id,'%06d') '_cam' num2str(i-1,'%03d')]);
    camera.setAttribute('sensor_id', sprintf('%i', i));
    camera.setAttribute('enabled', 'true');
    CameraNode = CamerasNode.appendChild(camera);
    
    transform = docNode.createElement('transform');
    transform_data_temp = [];
    for trans_idx = 1:16
        transform_data_temp = [transform_data_temp, sprintf('%f ', camera_trans_set{i}(trans_idx))];
    end
    transform.appendChild(docNode.createTextNode(transform_data_temp));
    ResolutionNode = CameraNode.appendChild(transform);
end

reference = docNode.createElement('reference');

reference_tempo = 'LOCAL_CS["Local Coordinates (m)",LOCAL_DATUM["Local Datum",0],UNIT["metre",1,AUTHORITY["EPSG","9001"]]]';
reference.appendChild(docNode.createTextNode(reference_tempo));
ReferenceNode = ChunkNode.appendChild(reference);


region = docNode.createElement('region');
RegionNode = ChunkNode.appendChild(region);


center = docNode.createElement('center');
center_data_temp = [];
for trans_idx = 1:3
    center_data_temp = [center_data_temp sprintf('%f ',center_set(trans_idx))];
end
center.appendChild(docNode.createTextNode(center_data_temp));
CenterNode = RegionNode.appendChild(center);


sizeEle = docNode.createElement('size');
size_data_temp = [];
for trans_idx = 1:3
    size_data_temp = [size_data_temp sprintf('%f ',size_set(trans_idx))];
end
sizeEle.appendChild(docNode.createTextNode(size_data_temp));
SizeNode = RegionNode.appendChild(sizeEle);


R_node = docNode.createElement('R');
R_data_temp = [];
for trans_idx = 1:9
    R_data_temp = [R_data_temp sprintf('%f ',R_set(trans_idx))];
end
R_node.appendChild(docNode.createTextNode(R_data_temp));
RNode = RegionNode.appendChild(R_node);

settings = docNode.createElement('settings');
SettingsNode = ChunkNode.appendChild(settings);

settings_property_1 = docNode.createElement('property');
SettingsNode.appendChild(settings_property_1);
settings_property_1.setAttribute('name','accuracy_tiepoints');
settings_property_1.setAttribute('value','1');


settings_property_2 = docNode.createElement('property');
SettingsNode.appendChild(settings_property_2);
settings_property_2.setAttribute('name','accuracy_cameras');
settings_property_2.setAttribute('value','3');


settings_property_3 = docNode.createElement('property');
SettingsNode.appendChild(settings_property_3);
settings_property_3.setAttribute('name','accuracy_cameras_ypr');
settings_property_3.setAttribute('value','2');


settings_property_4 = docNode.createElement('property');
SettingsNode.appendChild(settings_property_4);
settings_property_4.setAttribute('name','accuracy_markers');
settings_property_4.setAttribute('value','0.005');


settings_property_5 = docNode.createElement('property');
SettingsNode.appendChild(settings_property_5);
settings_property_5.setAttribute('name','accuracy_scalebars');
settings_property_5.setAttribute('value','0.001');

settings_property_6 = docNode.createElement('property');
SettingsNode.appendChild(settings_property_6);
settings_property_6.setAttribute('name','accuracy_projections');
settings_property_6.setAttribute('value','0.1');

save_dir = data_dir;
xmlwrite(save_dir + "\" + save_filename, docNode);

return