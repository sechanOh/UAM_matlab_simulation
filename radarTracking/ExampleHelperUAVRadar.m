classdef ExampleHelperUAVRadar < uav.SensorAdaptor
    %ExampleHelperUAVRadar Adapts the radarDataGenerator to UAV Scenario usage
    
    %   Copyright 2020-2023 The MathWorks, Inc.
    
    properties
        %UpdateRate Sensor Update Rate in Hz
        UpdateRate
    end
    
    methods
        function s = get.UpdateRate(obj)
            s = obj.SensorModel.UpdateRate;
        end
        
        function set.UpdateRate(obj, s)
            obj.SensorModel.UpdateRate = s;
        end
    end
    
    methods
        function obj = ExampleHelperUAVRadar(sensorModel)
            %uavRadar
            
            obj@uav.SensorAdaptor(sensorModel);
        end
        
        
        function setup(~, ~, ~)
            %setup Prepare sensor for simulation
            
            % For radar sensor, setup is no-op
        end
        
        
        function [dets,numDets,config] = read(obj, scene, egoPlatform, sensor, t)
            %read Generate sensor readings based on platforms in scene
            
            % Build inputs for the radarDataGenerator step methods, i.e
            % - targets : an array of struct with the following fields
            %             - PlatformID
            %             - ClassID
            %             - Position ( in scenario coordinates)
            %             - Velocity 
            %             - Acceleration
            %             - Orientation (rotation matrix or quaternion)
            %             - Angular Velocity
            %
            % - egoPose : a struct containing the pose of the mounting uav
            % - t : simulation time
            
            % Define pose of the ego uav
            egoPoseVector = obj.getMotion(scene, egoPlatform, sensor, t);
            egoPose = struct;
            egoPose.Position = egoPoseVector(1:3);
            egoPose.Velocity = egoPoseVector(4:6);
            egoPose.Orientation = quaternion(egoPoseVector(10:13));
            egoPose.AngularVelocity = zeros(1,3);

            targets = scene.targetPoses(egoPlatform);
                       
            [dets,numDets,config] = obj.SensorModel(targets,egoPose,t);
        end
        
        
        function out = getEmptyOutputs(~)
            %getNumOutput Provide outputs when sensor is not updated
            out = {nan, nan, nan};
        end
        
        
        function reset(obj)
            %reset Reset sensor states
            obj.SensorModel.reset();
        end
    end
end

