classdef helperLidarDetector < matlab.System
    % helperLidarDetector A helper class to segment the point cloud
    % into point detections and ground returns.
    % The step call to the object does the following things:
    %
    % 1. Removes point cloud outside the limits.
    % 2. From the survived point cloud, segments out ground.
    % 3. From the obstacle point cloud, forms clusters and output
    %    objectDetections of each cluster mean point.
    
    properties (Hidden)
        LidarMountingLocation = [0 0 0];
        LidarMountingAngles = [0 0 0];
    end
   
    % Ground Segmentation Properties
    properties
        %MaxWindowRadius Maximum radius size of structuring elements
        MaxWindowRadius = 3;
        %GridResolution
        GridResolution = 1.5;
    end
    
    % Segmentation properties
    properties
        % SegmentationMinDistance Distance threshold for segmentation
        SegmentationMinDistance = 5;
        % MinDetectionsPerCluster Minimum number of detections per cluster
        MinDetectionsPerCluster = 2;
        % MinZDistanceCluster Minimum Z-value of cluster in scenario axes
        MinZDistanceCluster = 20;
    end
    
    % Ego vehicle radius to remove ego vehicle point cloud.
    properties
        % EgoVehicleRadius Radius of ego vehicle
        EgoVehicleRadius = 10;
    end
    
    
    properties (Hidden)
        % MeasurementNoise Measurement noise for the bounding box detection
          MeasurementNoise = blkdiag(2*eye(3),7^2*eye(3),4*eye(4));
    end
    
    properties (Nontunable, Hidden)
        MeasurementParameters = struct.empty(0,1);
    end
    
    methods 
        function obj = helperLidarDetector(scene,varargin)
            setProperties(obj,nargin,varargin{:})
            % Read lidar config
            lidar = scene.Platforms(end).Sensors(1);
            obj.LidarMountingLocation = lidar.MountingLocation;
            obj.LidarMountingAngles = lidar.MountingAngles;
        end
    end
    
    methods (Access = protected)
        function [bboxDets,nonGroundPtCloud,groundPtCloud] = stepImpl(obj,egoPose, currentPointCloud,time)
            validPointCloud = removeInvalidPoints(currentPointCloud);
            % Crop point cloud
            pcCropped = cropPointCloud(validPointCloud,obj.EgoVehicleRadius);          
            % Convert point cloud to scenario coordinates
            pcCroppedScene = toScenario(egoPose,obj.LidarMountingLocation, obj.LidarMountingAngles, pcCropped);
            % Remove ground plane
            [~,nonGroundPtCloud,groundPtCloud] = segmentGroundSMRF(pcCroppedScene,obj.GridResolution,'MaxWindowRadius',obj.MaxWindowRadius);
            % Form clusters and get bounding boxes
            detBBoxes = getBoundingBoxes(nonGroundPtCloud,obj.SegmentationMinDistance,obj.MinDetectionsPerCluster,obj.MinZDistanceCluster);
            % Assemble detections
            if isempty(obj.MeasurementParameters)
                measParams = {};
            else
                measParams = obj.MeasurementParameters;
            end
            bboxDets = assembleDetections(detBBoxes,obj.MeasurementNoise,measParams,time);
        end
    end
end
    
function detections = assembleDetections(bboxes,measNoise,measParams,time)
% This method assembles the detections in objectDetection format.
numBoxes = size(bboxes,2);
detections = cell(numBoxes,1);
for i = 1:numBoxes
    detections{i} = objectDetection(time,cast(bboxes(:,i),'double'),...
        'MeasurementNoise',double(measNoise),'ObjectAttributes',struct,...
        'MeasurementParameters',measParams);
end
end

function bboxes = getBoundingBoxes(ptCloud,minDistance,minDetsPerCluster, minZDistance)
    % This method fits bounding boxes on each cluster
    % Cluster must have at least minDetsPerCluster points.
    % Its mean z must be above minZDistance. This allows a simple filtering of buildings.
    statedim = 10;
    
    [labels,numClusters] = pcsegdist(ptCloud,minDistance,'ParallelNeighborSearch',true);
    pointData = ptCloud.Location;
    bboxes = nan(statedim,numClusters,'like',pointData);
    isValidCluster = false(1,numClusters);
    for i = 1:numClusters
        thisPointData = pointData(labels == i,:);
        meanPoint = mean(thisPointData,1);
        if size(thisPointData,1) > minDetsPerCluster && meanPoint(3) > minZDistance
            
            % Cuboid model
            cuboid = pcfitcuboid(pointCloud(thisPointData));
            yaw = cuboid.Orientation(3);
            pitch = cuboid.Orientation(2);
            roll = cuboid.Orientation(1);
            q = quaternion([yaw pitch roll],'eulerd','zyx','frame');
            [q0, q1, q2, q3] = parts(q);
            L = cuboid.Dimensions(1);
            W = cuboid.Dimensions(2);
            H = cuboid.Dimensions(3);
            bboxes(:,i) = [cuboid.Center L W H q0 q1 q2 q3]';
            isValidCluster(i) = true;
        end
    end
    bboxes = bboxes(:,isValidCluster);
end


function ptCloudScenario = toScenario(egoPose,lidarLocation, lidarOrient, ptCloudEgo)
% This method transforms the coordinates of the lidar point cloud from ego
% to the scenario frame.

% egoPose is 1x16 vector including position, velocity, acceleration,  
% orientation, angular velocity as [x,y,z,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wx,wy,wz].

egoPos = egoPose(1:3);
egoOrient = quaternion(egoPose(10:13));
Rscene2ego = rotmat(egoOrient, 'frame');
Rego2lidar = rotmat(quaternion(lidarOrient,'eulerd','zyx','frame'),'frame');
Rscene2lidar = Rego2lidar * Rscene2ego;
locations = egoPos + lidarLocation * Rscene2ego + ptCloudEgo.Location * Rscene2lidar;
ptCloudScenario = pointCloud(locations);
end

function ptCloudOut = cropPointCloud(ptCloudIn,egoVehicleRadius)
    % This method removes the ego vehicle point cloud using 
    % findNeighborsInRadius
    
    nearIndices = findNeighborsInRadius(ptCloudIn,[0 0 0],egoVehicleRadius);
    nonEgoIndices = true(ptCloudIn.Count,1);
    nonEgoIndices(nearIndices) = false;
    validIndices = nonEgoIndices;
    indices = find(validIndices);
    ptCloudOut = select(ptCloudIn,indices);
end
