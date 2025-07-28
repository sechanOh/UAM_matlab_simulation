%% Simulate Simple Flight Scenario and Sensor in Unreal Engine Environment
% https://kr.mathworks.com/help/uav/ug/simulate-a-simple-flight-scenario-and-sensor-in-3d-environment.html

% Check requirements
required = ["UAV Toolbox", "Simulink", "Computer Vision Toolbox", "Simulink 3D Animation"];
checkToolboxes(required);

%% Open Simulink
open_system("uav_simple_flight_model.slx")

%% Show Image
imshow('USCityBlock.jpg',...
    'XData', [-250, 210],...
    'YData', [-225, 235]);
set(gca,'YDir','reverse')