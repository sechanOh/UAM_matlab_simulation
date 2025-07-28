%% Simulate UAV Mission in Urban Environment
% https://kr.mathworks.com/help/uav/ug/simulate-uav-mission-in-urban-environment.html

% Check requirements
required = ["UAV Toolbox"];
checkToolboxes(required);

%% 3D Mesh Map

scene = uavScenario(ReferenceLocation=[40.707088 -74.012146 0],UpdateRate=5);

xTerrainLimits = [-200 200];
yTerrainLimits = [-200 200];
color = [0.6 0.6 0.6];
addMesh(scene,"terrain",{"gmted2010",xTerrainLimits,yTerrainLimits},color)
xBuildingLimits = [-150 150];
yBuildingLimits = [-150 150];
color = [0.6431 0.8706 0.6275];
addMesh(scene,"buildings",{"manhattan.osm",xBuildingLimits,yBuildingLimits,"auto"},color)

plat = uavPlatform("UAV",scene);
updateMesh(plat,"quadrotor",{3},[1 0 0],eul2tform([0 0 pi]));

m = uavMission(PlanFile="orbitexample.plan");
figure
show3D(scene);
hold on
ax = show(m, ReferenceLocation=scene.ReferenceLocation);
missionLine = findobj(ax,"type","line");
missionText = findobj(ax,"type","text");
for idx = 1:numel(missionLine)
    missionLine(idx).LineWidth = 2;
end
for idx = 1:numel(missionText)
    p = missionText(idx).Position;
    missionText(idx).Position = p+[0 10 0];
    missionText(idx).HorizontalAlignment = "center";
    missionText(idx).VerticalAlignment = "bottom";
    missionText(idx).Margin = 1;
    missionText(idx).BackgroundColor = [1 1 1];
end
view([0 90])
hold off

%% UAV Simulation

parser = multirotorMissionParser;
traj = parse(parser,m,scene.ReferenceLocation);

figure
ax = show3D(scene);
light(ax,Position=[-800 -800 800])
view([-80 80])
hold on
show(m,ReferenceLocation=scene.ReferenceLocation);
missionLine = findobj(ax,"type","line","tag","PathLine");
missionLine.LineWidth = 2;
missionText = findobj(ax,"type","text");
for idx = 1:numel(missionText)
    p = missionText(idx).Position;
    missionText(idx).Position = p+[0 10 0];
    missionText(idx).HorizontalAlignment = "center";
    missionText(idx).VerticalAlignment = "bottom";
    missionText(idx).FontWeight = "bold";
end
show(traj,NumSamples=200);

camzoom(ax,10)
setup(scene)
while scene.CurrentTime <= traj.EndTime
    motion = query(traj,scene.CurrentTime);
    move(plat,motion);
    show3D(scene,Parent=ax,FastUpdate=true);
    nedPos = motion(1:3);
    camtarget(ax,[nedPos(2) nedPos(1) -nedPos(3)]);
    advance(scene);
    drawnow limitrate
end
hold off

