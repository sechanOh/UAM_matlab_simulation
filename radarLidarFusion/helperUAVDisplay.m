classdef helperUAVDisplay < matlab.System
    %helperUAVDisplay helper class for visualization of UAV tracking
    %scenarios
    
    properties
        RadarColor =            [0         0.4470    0.7410];
        LidarColor =            [0.8500    0.3250    0.0980];
        FusedColor =            [0.4940    0.1840    0.5560];
        PointCloudColor =       [0.9290    0.6940    0.1250];
        PointCloudGroundColor = [0.7         0.5       0.1];
        BuildingColor =         [0.2314    0.6000    0.1490];
        GroundColor =           [0.8       0.8       0.8];
    end
    
    properties
        LidarTrackPositionSelector = [1 0 0 0 0 0 0 0 0 0 0 0 0;...
            0 0 1 0 0 0 0 0 0 0 0 0 0; ...
            0 0 0 0 1 0 0 0 0 0 0 0 0];
        LidarTrackVelocitySelector = [0 1 0 0 0 0 0 0 0 0 0 0 0;...
            0 0 0 1 0 0 0 0 0 0 0 0 0; ...
            0 0 0 0 0 1 0 0 0 0 0 0 0];
        RadarTrackPositionSelector = [1 0 0 0 0 0;...
            0 0 1 0 0 0; ...
            0 0 0 0 1 0];
        RadarTrackVelocitySelector = [0 1 0 0 0 0;...
            0 0 0 1 0 0; ...
            0 0 0 0 0 1];
    end
    
    properties (Hidden)
        Figure
        Layout
        %Axes axes in layout following order
        % Scene3D Scene2D UAVA UAVB UAVC
        Axes
        LegendTile
        Scenario
        PlotFrames
        
        XLim = [1790 2000];
        YLim = [40 240];
        ZLim = [0 80];
        ZoomLength = [10 10 10; 20 20 20; 10 10 10]*0.8;
        
        Frames = cell(0,4);
        
        RecordGIF = false;
        Map
    end
    
    properties (Access = protected)
        RadarCoveragePlotters fusion.theaterPlot.CoveragePlotter
        LidarCoveragePlotters fusion.theaterPlot.CoveragePlotter
        RadarDetectionPlotters matlab.graphics.chart.primitive.Line
        LidarDetectionPlotters matlab.graphics.chart.primitive.Line
        RadarTrackPlotters fusion.theaterPlot.TrackPlotter
        LidarTrackPlotters fusion.theaterPlot.TrackPlotter
        FusedTrackPlotters fusion.theaterPlot.TrackPlotter
        LidarPointCloud matlab.graphics.chart.primitive.Line
        LidarPointCloudGround matlab.graphics.chart.primitive.Line
        PlatformFollowList = [2 3 4 1];
    end
    
    methods
        
        function obj = helperUAVDisplay(scene,varargin)
            setProperties(obj,varargin{:});
            createFigureLayout(obj);
            loadScene(obj,scene);
            updateScene(obj);
            createPlotters(obj);
            createLegend(obj);
            setupCamera(obj);
            customizeAxes(obj);
            obj.Figure.Visible = 'on';
        end
        
        function createGIFs(obj)
            % get right color map once
            [~,obj.Map] = rgb2ind(obj.Frames{end,1}.cdata,256,'nodither');
            writeAnimation(obj,2,1:60,1,0,'UAVA',0.75);
            writeAnimation(obj,3,50:100,1,0.3,'UAVB',0.75);
            writeAnimation(obj,4,30:75,1,0.3,'UAVC',0.75);
        end
    end
    
    
    methods(Access = protected)
        
        function stepImpl(obj, radarcov, lidarcov, ptCloud, groundCloud, lidardets, radardets, lidartracks, radartracks, fusedtracks )
            updateScene(obj);
            plotCoverage(obj,radarcov,lidarcov);
            plotPointCloud(obj,groundCloud, ptCloud);
            plotLidarDetections(obj, lidardets);
            plotRadarDetections(obj, radardets);
            plotLidarTracks(obj, lidartracks);
            plotRadarTracks(obj, radartracks);
            plotFusedTracks(obj, fusedtracks);
            drawnow limitrate
            
            if obj.RecordGIF
                newframes = cell(1,4);
                for i = 1:numel(obj.Axes)
                    newframes{i} = getframe(obj.Axes(i));
                end
                obj.Frames(end+1,:) = newframes;
            end
        end
        
        function createFigureLayout(obj)
            
            f = figure('NumberTitle','off',...
                'Name','UAV-borne Lidar and Radar fusion',...
                'Visible','off',...
                'Color','black',...
                'Position',[200 100 1450 860]);
            %tiledlayout do not allow enough customization -- use panels
            
            %             t = tiledlayout(f,2,3, 'TileSpacing','tight','Padding','none');
            %             scene3d = nexttile(1,[1 2]);
            %             %             scene2d = nexttile(2);
            %             uavA = nexttile(4);
            %             title(uavA, 'UAV A');
            %             uavB = nexttile(5);
            %             title(uavB, 'UAV B');
            %             uavC = nexttile(6);
            %             title(uavC, 'UAV C');
            %             legax = nexttile(3);
            %             legax.Visible = 'off';
            
            
            % Add panels
            scene3dpanel = uipanel(f,'Position',[0 1/3 1 2/3],'BorderType','beveledout','FontSize',12,'BackgroundColor','white');
            legendpanel = uipanel(f,'Position',[3/4 0 1/4 1/3],'BorderType','beveledout','FontSize',12,'BackgroundColor','white');
            uavApanel = uipanel(f,'Position',[0 0 1/4 1/3],'BorderType','beveledout','FontSize',12,'BackgroundColor','white');
            uavBpanel = uipanel(f,'Position',[1/4 0 1/4 1/3],'BorderType','beveledout','FontSize',12,'BackgroundColor','white');
            uavCpanel = uipanel(f,'Position',[2/4 0 1/4 1/3],'BorderType','beveledout','FontSize',12,'BackgroundColor','white');
            
            % Create axes
            scene3d = axes(scene3dpanel,'Position',[-0.02 -0.8 1.5 2.85],'Color','white');
            scene3dpanel.Title =  'Scene';
            uavA = axes(uavApanel,'Box','off','Position',[-0.1 -0.2 1.1 1.5],'Color','white');
            uavApanel.Title = 'UAV A';
            uavB = axes(uavBpanel,'Box','off','Position',[-0.1 -0.2 1.1 1.5],'Color','white');
            uavBpanel.Title =  'UAV B';
            uavC = axes(uavCpanel,'Box','off','Position',[-0.1 -0.2 1.1 1.5],'Color','white');
            uavCpanel.Title = 'UAV C';
            legendpanel.Title = 'Legend';
            legax = axes(legendpanel);
            legax.Visible = 'off';
            
            %
            obj.Figure = f;
            %                         obj.Layout = t;
            obj.Axes = [scene3d;...
                ...scene2d;...
                uavA;...
                uavB;...
                uavC];
            obj.LegendTile = legax;
            
        end
        
        function customizeAxes(obj)
            
            % Remove axis for chase views
            for i=1:4
                ax = obj.Axes(i);
                ax.XAxis.Visible = 'off';
                ax.YAxis.Visible = 'off';
                ax.ZAxis.Visible = 'off';
                box(ax,'off');
            end
            
            % Add labels for scene view
            xlabel(obj.Axes(1),'East (m)');
            ylabel(obj.Axes(1),'North (m)');
            zlabel(obj.Axes(1),'Up (m)');
            
            % disable a few things
            for i=1:numel(obj.Axes)
                ax = obj.Axes(i);
                disableDefaultInteractivity(ax);
                ax.PositionConstraint = 'innerposition';
            end
            
        end
        
        function loadScene(obj, scene)
            obj.Scenario = scene;
            for i=1:numel(obj.Axes)
                ax = obj.Axes(i);
                [~, frame] = show3D(scene,'Parent',ax);
                if i==1
                    obj.PlotFrames = frame;
                else
                    obj.PlotFrames(i) = frame;
                end
                hold(ax,'on')
                
                % Modify default colors
                allpatches = findobj(ax.Children,'flat','Type','Patch');
                set(allpatches(1:end-1),'FaceAlpha',1,'FaceColor',obj.BuildingColor,...
                    'EdgeColor',0.7*obj.BuildingColor);
                gr = allpatches(end);
                set(gr,'FaceColor', obj.GroundColor,'EdgeColor', [0.5 0.5 0.5]);
            end
        end
        
        function setupCamera(obj)
            % Setup limits and view angles for scene axes
            scene3d =obj.Axes(1);
            xlim(scene3d,obj.XLim);
            ylim(scene3d,obj.YLim);
            zlim(scene3d,obj.ZLim);
            view(scene3d, 70,10);
            %             camva(scene3d, 5);
            %             campos(scene3d,  [2319   -123.25    342.3]);
            
            %             scene2d = obj.Axes(2);
            %             view(scene2d, 2);
            
            for i=2:4
                view(obj.Axes(i), -15,30);
            end
        end
        
        function updateScene(obj)
            scene = obj.Scenario;
            for i=1:numel(obj.Axes)
                ax = obj.Axes(i);
                show3D(scene,'FastUpdate',true,'Parent',ax);
            end
            
            % Move camera
            updateCamera(obj);
        end
        
        function updateCamera(obj)
            % Axes following a target needs to update their limits
            platforms = obj.Scenario.Platforms(1:end-1); % assumes EGO at the end
            time = obj.Scenario.CurrentTime;
            for i=1:numel(platforms)
                pos = lookupPose(platforms(i).Trajectory,time);
                centerLimits(obj, obj.PlatformFollowList(i), pos);
                % No view angle change
            end
            
            %Optinally follow ego
            %             pos = lookupPose(obj.Scenario.Platforms(end).Trajectory,time);
            %             centerLimits(obj, obj.PlatformFollowList(end), pos, [200 200 150]);
        end
        
        function centerLimits(obj, i, pos, span)
            ax = obj.Axes(i);
            if nargin == 3
                span = obj.ZoomLength(i-1,:);
            end
            if ~any(isnan(pos))
                xlim(ax, [pos(1)-span(1)/2, pos(1) + span(1)/2]);
                ylim(ax, [pos(2)-span(2)/2, pos(2) + span(2)/2]);
                zlim(ax, [pos(3) - span(3)/2 pos(3) + span(3)/2]);
            end
        end
        
        function createPlotters(obj)
            % Add theaterPlot plotters to each axis ENU frame
            
            for i=1:numel(obj.Axes)
                hold(obj.Axes(i),'on');
                tp = theaterPlot('Parent',obj.Axes(i));
                if i < 2 % only had coverage on scene views
                    obj.RadarCoveragePlotters(i) = coveragePlotter(tp,'DisplayName','','Color',obj.RadarColor,...
                        'Alpha',[0.1 0.1]);
                    obj.LidarCoveragePlotters(i) = coveragePlotter(tp,'DisplayName','','Color',obj.PointCloudColor,...
                        'Alpha',[0.1 0.1]);                
                end
                
                obj.RadarDetectionPlotters(i) = plot3(obj.Axes(i),...
                    nan,nan,nan,...
                    'LineStyle','none',...
                    'Marker','+','MarkerSize',13,...
                    'MarkerEdgeColor',obj.RadarColor);
                
                obj.LidarDetectionPlotters(i) = plot3(obj.Axes(i),...
                    nan,nan,nan,...
                    'LineStyle','none',...
                    'Marker','o','MarkerSize',12,...
                    'MarkerEdgeColor',obj.LidarColor);
                
                obj.RadarTrackPlotters(i) = trackPlotter(tp,'DisplayName','',...
                    'MarkerEdgeColor',obj.RadarColor,'MarkerFaceColor',obj.RadarColor,...
                    'MarkerSize',10,'ConnectHistory','on','HistoryDepth',1500,...
                    'LabelOffset',[1 0 0]);
                obj.LidarTrackPlotters(i) = trackPlotter(tp,'DisplayName', '',...
                    'ConnectHistory','on','HistoryDepth',1500,'MarkerEdgeColor',obj.LidarColor,...
                    'MarkerFaceColor',obj.LidarColor,...
                    'LabelOffset',[0 1 0]);
                obj.FusedTrackPlotters(i) = trackPlotter(tp,'DisplayName', '',...
                    'ConnectHistory','on','HistoryDepth',1500,'MarkerEdgeColor',obj.FusedColor,...
                    'MarkerFaceColor',obj.FusedColor,...
                    'LabelOffset',[0 0 1]);
                obj.LidarPointCloud(i) = plot3(obj.Axes(i),...obj.PlotFrames(i).EgoVehicle.Lidar,...
                    nan,nan,nan,...
                    'LineStyle','none',...
                    'Marker','.','MarkerSize',8,'MarkerEdgeColor',obj.PointCloudColor);
                obj.LidarPointCloudGround(i) = plot3(obj.Axes(i),...obj.PlotFrames(i).EgoVehicle.Lidar,...
                    nan,nan,nan,...
                    'LineStyle','none',...
                    'Marker','.','MarkerSize',5,...
                    'MarkerEdgeColor',obj.PointCloudGroundColor);
                legend(obj.Axes(i),'Off');
            end
        end
        
        function createLegend(obj)
            % Create plotters to legend tile just for the legend
            hold(obj.LegendTile,'on');
            plot3(obj.LegendTile,...
                nan,nan,nan,...
                'LineStyle','none',...
                'Marker','.','MarkerSize',8,'MarkerEdgeColor',obj.PointCloudColor,...
                'DisplayName','Point Cloud');
            plot3(obj.LegendTile,...
                nan,nan,nan,...
                'LineStyle','none',...
                'Marker','.','MarkerEdgeColor',obj.PointCloudGroundColor,...
                'DisplayName','Point Cloud Ground Segment');
            
            tp = theaterPlot('Parent',obj.LegendTile);
            platformPlotter(tp,'DisplayName','Lidar Detections',...
                'Marker','o','MarkerEdgeColor',obj.LidarColor);
            detectionPlotter(tp,'DisplayName','Radar Detections',...
                'Marker','+','MarkerEdgeColor',obj.RadarColor,...
                'MarkerSize',13);
            trackPlotter(tp,'DisplayName','Lidar Tracks',...
                'MarkerEdgeColor',obj.LidarColor,'MarkerFaceColor',obj.LidarColor,...
                'MarkerSize',10);
            
            trackPlotter(tp,'DisplayName','Radar Tracks',...
                'MarkerEdgeColor',obj.RadarColor,'MarkerFaceColor',obj.RadarColor,...
                'MarkerSize',10);
            
            platformPlotter(tp,'DisplayName','Fused Tracks',...
                'MarkerEdgeColor',obj.FusedColor,'MarkerFaceColor',obj.FusedColor,...
                'MarkerSize',10);
            cp = coveragePlotter(tp,'DisplayName','Radar FOV','Color',obj.RadarColor);
            cp2 = coveragePlotter(tp,'DisplayName','Lidar FOV','Color',obj.PointCloudColor);

            l = legend(obj.LegendTile);
            l.Location = 'best';
            
            % coverage legend patch updates its color after first call
            cov = coverageConfig(obj.Scenario.Platforms(end).Sensors(2).SensorModel);
            plotCoverage(cp, cov);
            plotCoverage(cp2, cov);
            % legend is all set. Turn off auto update
            l.AutoUpdate = 'off';
            
            % Hide axis
            obj.LegendTile.Visible = 'off';
            p = findobj(obj.LegendTile,'Tag','coverageFrame1');
            set(p,'Visible','off');
            obj.LegendTile.Toolbar.Visible = 'off';
        end
        
        function plotCoverage(obj, cov, lidarcov)
            % Add geoPose information
            egoPose = read(obj.Scenario.Platforms(end));
            cov.Position(:) = cov.Position + egoPose(1:3);
            cov.Orientation = quaternion(egoPose(10:13)) * cov.Orientation;

            lidarcov.Position(:) = lidarcov.Position + egoPose(1:3);
            lidarcov.Orientation = quaternion(egoPose(10:13)) * lidarcov.Orientation;

            for i=1:1
                plotCoverage(obj.RadarCoveragePlotters(i),cov)
                plotCoverage(obj.LidarCoveragePlotters(i),lidarcov)
            end
        end
        
        function plotLidarTracks(obj, tracks)
            if numel(tracks)>0
                possel = obj.LidarTrackPositionSelector;
                velsel = obj.LidarTrackVelocitySelector;
                [pos, vel, dim, orient, labels] = obj.plotLidarTrackDescriptors(tracks, possel, velsel);
                showTrack = zeros(1,numel(tracks),'logical');
                for i=1:numel(obj.Axes)
                    % Only plot tracks if they are within limits
                    xLim = obj.Axes(i).XLim + [-5 5];
                    yLim = obj.Axes(i).YLim + [-5 5];
                    zLim = obj.Axes(i).ZLim + [- 5 5];
                    for j =1:numel(tracks)
                        if pos(j,1) < xLim(2) && pos(j,1) > xLim(1) && ...
                                pos(j,2) < yLim(2) && pos(j,2) > yLim(1) && ...
                                pos(j,3) < zLim(2) && pos(j,3) > zLim(1)
                            showTrack(j) = true;
                        end
                    end
                    if any(showTrack)
                        plotTrack(obj.LidarTrackPlotters(i),pos(showTrack,:),vel(showTrack,:),labels(showTrack), dim(showTrack), orient(showTrack));
                    end
                end
            end
        end
        
        function plotRadarTracks(obj, tracks)
            if numel(tracks)>0
                possel = obj.RadarTrackPositionSelector;
                velsel = obj.RadarTrackVelocitySelector;
                [pos, vel, cov, labels] = obj.plotRadarTrackDescriptors(tracks,possel,velsel);
                showTrack = zeros(1,numel(tracks),'logical');
                for i=1:numel(obj.Axes)
                    % Only plot tracks if they are within limits
                    xLim = obj.Axes(i).XLim + [-5 5];
                    yLim = obj.Axes(i).YLim + [-5 5];
                    zLim = obj.Axes(i).ZLim + [- 5 5];
                    for j =1:numel(tracks)
                        if pos(j,1) < xLim(2) && pos(j,1) > xLim(1) && ...
                                pos(j,2) < yLim(2) && pos(j,2) > yLim(1) && ...
                                pos(j,3) < zLim(2) && pos(j,3) > zLim(1)
                            showTrack(j) = true;
                        end
                    end
                    if any(showTrack)
                        plotTrack(obj.RadarTrackPlotters(i),pos(showTrack,:),vel(showTrack,:),cov(:,:,showTrack),labels(showTrack));
                    end
                end
            end
        end
        
        function plotRadarDetections(obj, dets)
            if numel(dets)>0
                detpos = obj.plotRadarDetectionDescriptors(dets);
                for i=1:numel(obj.Axes)
                    %                     plotDetection(obj.RadarDetectionPlotters(i), detpos');
                    set(obj.RadarDetectionPlotters(i),'XData',detpos(1,:), 'YData', detpos(2,:), 'ZData',detpos(3,:));
                end
            else
                for i=1:numel(obj.Axes)
                    set(obj.RadarDetectionPlotters(i),'XData',nan, 'YData', nan, 'ZData',nan);
                end
            end
        end
        
        function plotLidarDetections(obj,dets)
            if numel(dets)>0
                [detpos, dim, orient] = obj.plotLidarDetectionDescriptors(dets); %#ok<ASGLU>
                for i=1:numel(obj.Axes)
                    % dim and orientation are expensive to plot
                    %                     plotPlatform(obj.LidarDetectionPlotters(i), detpos, dim, orient);
                    set(obj.LidarDetectionPlotters(i),'XData',detpos(:,1), 'YData', detpos(:,2), 'ZData',detpos(:,3));
                end
            else
                for i=1:numel(obj.Axes)
                    set(obj.LidarDetectionPlotters(i),'XData',nan, 'YData',nan, 'ZData',nan);
                end
            end
            
        end
        
        function plotPointCloud(obj, groundCloud, nonGroundCloud)
            [x,y,z] = obj.plotPointCloudDescriptors(nonGroundCloud);
            [xg,yg,zg] = obj.plotPointCloudDescriptors(groundCloud);
            for i=1:numel(obj.Axes)
                set(obj.LidarPointCloud(i),'XData', x, 'YData', y, 'ZData', z);
                set(obj.LidarPointCloudGround(i),'XData', xg, 'YData', yg, 'ZData', zg);
            end
        end
        
        function plotFusedTracks(obj, tracks)
            possel = obj.LidarTrackPositionSelector;
            velsel = obj.LidarTrackVelocitySelector;
            if numel(tracks)>0
                [pos, vel, dim, orient, labels] = obj.plotFusedTrackDescriptors(tracks, possel, velsel);
                showTrack = zeros(1,numel(tracks),'logical');
                for i=1:numel(obj.Axes)
                    % Only plot tracks if they are within limits
                    xLim = obj.Axes(i).XLim + [-5 5];
                    yLim = obj.Axes(i).YLim + [-5 5];
                    zLim = obj.Axes(i).ZLim + [- 5 5];
                    for j =1:numel(tracks)
                        if pos(j,1) < xLim(2) && pos(j,1) > xLim(1) && ...
                                pos(j,2) < yLim(2) && pos(j,2) > yLim(1) && ...
                                pos(j,3) < zLim(2) && pos(j,3) > zLim(1)
                            showTrack(j) = true;
                        end
                    end
                    if any(showTrack)
                        plotTrack(obj.FusedTrackPlotters(i),pos(showTrack,:),vel(showTrack,:),labels(showTrack), dim(showTrack), orient(showTrack));
                    end
                end
            end
        end
        
        function writeAnimation(obj,axIndex,frameIndices,dsFactor,delay,fName,scale)
            if nargin == 6
                scale = 1;
            end
            
            if obj.RecordGIF
                frames = obj.Frames(frameIndices,axIndex);
                imSize = size(imresize(frames{1}.cdata,scale));
                im = zeros(imSize(1),imSize(2),1,floor(numel(frames)/dsFactor),'uint8');
                count = numel(frames);
                for i = numel(frames):-dsFactor:1
                    thisImage = frames{i}.cdata;
                    thisImage = imresize(thisImage,scale);
                    thisImage = imresize(thisImage,imSize(1:2));
                    if isempty(obj.Map)
                        [im(:,:,1,count),obj.Map] = rgb2ind(thisImage,256,'nodither');
                    else
                        im(:,:,1,count) = rgb2ind(thisImage,obj.Map,'nodither');
                    end
                    count = count - 1;
                end
                imwrite(im,obj.Map,[fName,'.gif'],'DelayTime',delay,'LoopCount',inf);
            end
        end
    end
    
    methods (Static)
        function [x,y,z] = plotPointCloudDescriptors(ptcloud)
            x = ptcloud.Location(:,1);
            y = ptcloud.Location(:,2);
            z = ptcloud.Location(:,3);
        end
        
        function [pos, dim, orient] = plotLidarDetectionDescriptors(dets)
            %Return descriptors to plot lidar bounding box detections
            if iscell(dets)
                dets = [dets{:}];
            end
            
            ndet = numel(dets);
            pos = zeros(ndet,3);
            dim = repmat(struct('Length',0,'Width',0,'Height',0,'OriginOffset',[0 0 0]),ndet, 1);
            orient = repmat(quaternion(1,0,0,0),ndet,1);
            
            for i=1:ndet
                meas = dets(i).Measurement;
                pos(i,:) = meas(1:3);
                dim(i).Length = meas(4);
                dim(i).Width =meas(5);
                dim(i).Height = meas(6);
                orient(i) = quaternion(meas(7:end)');
            end
        end
        
        function  detpos = plotRadarDetectionDescriptors(dets)
            if iscell(dets)
                dets = [dets{:}];
            end
            detpos = [dets.Measurement];
            detpos = detpos(1:3,:);
        end
        
        function [pos, vel, cov, labels] = plotRadarTrackDescriptors(tracks, possel, velsel)
            % Show track elliptical extent via covariance
            n = numel(tracks);
            cov = zeros(3,3,n);
            labels = cell(n,1);
            pos = getTrackPositions(tracks, possel);
            vel = getTrackVelocities(tracks, velsel);
            for i=1:n
                cov(:,:,i) = tracks(i).Extent;
                labels{i} = ['R', num2str(tracks(i).TrackID)];
            end
        end
        
        function [pos, vel, dim , orient, labels] = plotLidarTrackDescriptors(tracks, possel, velsel)
            
            pos = getTrackPositions(tracks, possel);
            vel = getTrackVelocities(tracks, velsel);
            
            n = numel(tracks);
            dim = repmat(struct('Length',0,'Width',0,'Height',0,'OriginOffset',[0 0 0]),n, 1);
            orient = repmat(quaternion(1,0,0,0),n,1);
            labels = cell(n,1);
            for i=1:n
                state = tracks(i).State;
                dim(i).Length = state(7);
                dim(i).Width =state(8);
                dim(i).Height = state(9);
                orient(i) = quaternion(state(10:end)');
                labels{i} = ['L', num2str(tracks(i).TrackID)];
            end
        end
        
        function [pos, vel, dim , orient, labels] = plotFusedTrackDescriptors(tracks, possel, velsel)
            
            pos = getTrackPositions(tracks, possel);
            vel = getTrackVelocities(tracks, velsel);
            
            n = numel(tracks);
            dim = repmat(struct('Length',0,'Width',0,'Height',0,'OriginOffset',[0 0 0]),n, 1);
            orient = repmat(quaternion(1,0,0,0),n,1);
            labels = cell(n,1);
            for i=1:n
                state = tracks(i).State;
                dim(i).Length = state(7);
                dim(i).Width =state(8);
                dim(i).Height = state(9);
                orient(i) = quaternion(state(10:end)');
                labels{i} = ['F', num2str(tracks(i).TrackID)];
            end
        end
    end
end