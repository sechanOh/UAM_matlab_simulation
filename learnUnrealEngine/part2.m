%% Run a Vehicle Dynamics Maneuver in 3D Environment
% https://kr.mathworks.com/help/vdynblks/ug/run-a-maneuver-in-3d-environment.html

% Check requirements
required = ["Vehicle Dynamics Blockset", "Vehicle Dynamics Blockset Interface for Unreal Engine Projects"];
checkToolboxes(required);

%%

path = "C:\Users\SechanOh\Documents\MATLAB\AutoVrtlEnv\AutoVrtlEnv.uproject";
editor = sim3d.Editor(path);
open(editor);
