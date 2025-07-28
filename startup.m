%% startup.m
clc;

projectRoot = fileparts(mfilename('fullpath'));
[~, projectName] = fileparts(projectRoot);

disp(projectName + " setup started...");

addpath(genpath(projectRoot));
disp("Project path added:");
disp("   " + projectRoot);

disp(projectName + " setup completed.");

% fprintf('\nAvailable executables:\n');
% 
% targetFolderList = ["scripts", "tests"];
% for folderIdx = 1:length(targetFolderList)
%     folderName = targetFolderList(folderIdx);
%     scriptFolder = fullfile(projectRoot, folderName);
%     scriptFiles = dir(fullfile(scriptFolder, '*.m'));
%     for scriptFileIndex = 1:length(scriptFiles)
%         scriptName = erase(scriptFiles(scriptFileIndex).name, '.m');
%         fprintf('   ▶ <a href="matlab:run(''%s'')">run %s</a>\n', ...
%             scriptName, scriptName);
%     end
% end

clear;

