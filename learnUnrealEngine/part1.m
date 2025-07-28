%% Run a Vehicle Dynamics Maneuver in 3D Environment
% https://kr.mathworks.com/help/vdynblks/ug/run-a-maneuver-in-3d-environment.html

% Check requirements
required = ["Vehicle Dynamics Blockset", "Automated Driving Toolbox"];
checkToolboxes(required);

%% Simulate example

% you need to modify Visualization - 3D Engine as enabled.
vdynblksDblLaneChangeStart;
