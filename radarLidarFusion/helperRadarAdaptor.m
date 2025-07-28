classdef helperRadarAdaptor < uav.SensorAdaptor
    % Implementation of radarDataGenerator adapted for use in a UAV
    % scenario.
    
    %   Copyright 2021-2023 The MathWorks, Inc.
    
    properties
        %UpdateRate Sensor update rate
        % Use nan to update at the scenario's update rate.
        UpdateRate = nan
    end
    
    properties
        %MeshNRCS Normalized radar cross section of mesh faces (dB)
        MeshNRCS (1,1) double = -20; 
        
    end
    
    properties (Hidden)
        MeshPlatformPositions (:,3) double
        PlatformClassID (1,1) double = 1;
        MeshClassID (1,1) double = 5;
        IsEgo (1,:) logical
    end
    
    methods
        function obj = helperRadarAdaptor(sensorModel)
            
            obj@uav.SensorAdaptor(sensorModel);
            obj.UpdateRate = sensorModel.UpdateRate;
        end

        function setup(obj, scene, ~)
            %setup

            obj.UpdateRate = obj.SensorModel.UpdateRate;
            
            % Number of platforms in the scene
            platforms = scene.Platforms;
            obj.IsEgo = false(1, numel(platforms));
            
            % Add profile information for targets
            targetIdx = 1;
            for i = 1:numel(platforms)
                target = platforms(i);
                if ~strcmp(target.Name, "EgoVehicle")
                    profile = platformProfile(obj, target);
                    profile.PlatformID = i;
                    obj.SensorModel.Profiles(targetIdx) = profile;
                    targetIdx = targetIdx + 1;
                else
                    obj.IsEgo(i) = true;
                end
            end
        end
        
        function profile = platformProfile(obj, plat)
            platMesh = plat.Mesh;
            sig = mesh2RCS(obj,platMesh );
            dimension = obj.boundingBoxDimensions(platMesh); % Dimensions field used to determine if point tgt
            profile = struct('PlatformID',0,'ClassID',obj.PlatformClassID,...
                'Dimensions',dimension,'Signatures',{{sig}});
        end
        
        function [dets,numDets,config] = read(obj, scene, egoPlatform, sensor, t)
            %read 

            % Define pose of the ego uav
            egoPoseVector = obj.getMotion(scene, egoPlatform, sensor, t);
            egoPose = struct;
            egoPose.Position = egoPoseVector(1:3);
            egoPose.Velocity = egoPoseVector(4:6);
            egoPose.Orientation = quaternion(egoPoseVector(10:13));
            egoPose.AngularVelocity = zeros(1,3);
            
            % Setup targets to only track the non-ego platforms
            targets = scene.targetPoses(egoPlatform);
            targets = targets(1:numel(scene.Platforms)-1);

            [dets,numDets,config] = obj.SensorModel(targets,t);
            dets = obj.toScenario(egoPose, dets);
        end
        
        function sig  = mesh2RCS(obj, M)
            % Compute an RCS profile for the platform mesh
                freq = obj.SensorModel.CenterFrequency;
                az = -180:4:180;
                el = -90:4:90;
                nrcs = obj.MeshNRCS;
                sig = obj.getMeshRcsSig( M,az,el,nrcs,freq );
        end
        
        function reset(obj)
            reset(obj.SensorModel);
        end
        
        function out = getEmptyOutputs(~)
            %NO OP
            out = {nan nan nan};
        end
    end
    
    methods (Static)
        function D = boundingBoxDimensions(mesh)
            
            D.Length = max(mesh.Vertices(:,1)) - min(mesh.Vertices(:,1));
            D.Width  = max(mesh.Vertices(:,2)) - min(mesh.Vertices(:,2));
            D.Height = max(mesh.Vertices(:,3)) - min(mesh.Vertices(:,3));
            
            D.OriginOffset = [0 0 0];
            
        end
        
        function [sig,a] = getMeshRcsSig(M,az,el,nrcs,freq) 
            az = az(:).';
            el = el(:);
            % make sure mesh is centered
            center = (min(M.Vertices,[],1) + max(M.Vertices,[],1))/2;
            vertices = M.Vertices - center;
            
            v1 = vertices(M.Faces(:,1),:);
            v2 = vertices(M.Faces(:,2),:);
            v3 = vertices(M.Faces(:,3),:);
            
            w1 = v2 - v1;
            w2 = v3 - v1;
            
            n = [w1(:,2).*w2(:,3) - w2(:,2).*w1(:,3),...
                w2(:,1).*w1(:,3) - w1(:,1).*w2(:,3),...
                w1(:,1).*w2(:,2) - w2(:,1).*w1(:,2)];
            
            a = sqrt(sum(n.^2,2));
            n = n ./ a; % normals
            a = a / 2;
            c = (v1+v2+v3)/3; % centers
            
            farFieldRange = 2e2;
            lambda = 299792458 / freq;
            
            rcs = zeros(numel(el),numel(az));
            for azind = 1:numel(az)
                for elind = 1:numel(el)
                    
                    [src(1),src(2),src(3)] = sph2cart(az(azind)*pi/180,el(elind)*pi/180,farFieldRange);
                    
                    los = src - c;
                    R = sqrt(sum(los.^2,2));
                    los = los./R;
                    
                    A = a.*sum(n.*los,2)/lambda;
                    I = find(A>0);
                    A = A(I);
                    R = R(I);
                    
                    rcs(elind,azind) = sum(A.*exp(1i*2*pi/lambda*R));
                    
                end
            end
            
            rcs = 10^(nrcs/10)*abs(rcs);
            
            rcs(rcs == 0) = eps;
            
            if numel(el) == 1
                sig = rcsSignature('Azimuth',az,'Elevation',[el; el],'Pattern',10*log10(rcs));
            else
                sig = rcsSignature('Azimuth',az,'Elevation',el,'Pattern',10*log10(rcs));
            end
        end
        
        function detsout = toScenario(egoPose, dets)
            detsout = dets;
            for i=1:numel(dets)
                adet = dets{i};
                params = adet.MeasurementParameters(1);
%                 sensorOrigin = [params.OriginPosition params.OriginVelocity];
                sensorOrigin = [params.OriginPosition];
                Rsensor2ego = params.Orientation;
%                 Rsensor2ego = blkdiag(Rsensor2ego, Rsensor2ego);
                measEgo = sensorOrigin(:) + Rsensor2ego*adet.Measurement(:);
                
                Rego2scene = rotmat(egoPose.Orientation,'frame')';
%                 Rego2scene = blkdiag(Rego2scene, Rego2scene);
%                 egoOrigin = [egoPose.Position, egoPose.Velocity];
                egoOrigin = [egoPose.Position];
                measScene = egoOrigin(:) + Rego2scene* measEgo;
                
                Rsensor2scene = Rego2scene*Rsensor2ego;
                covScene = Rsensor2scene*adet.MeasurementNoise*Rsensor2scene';
                detsout{i}.Measurement = measScene;
                detsout{i}.MeasurementNoise = covScene;
                detsout{i}.MeasurementParameters.Orientation = eye(3);
                detsout{i}.MeasurementParameters.OriginPosition = zeros(1,3);
%                 detsout{i}.MeasurementParameters.OriginVelocity = zeros(1,3);
            end
        end
    end
end

