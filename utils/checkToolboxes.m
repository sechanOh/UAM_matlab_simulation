function checkToolboxes(required)

    if required == "update"
        if isfile('./utils/toolbox_map.mat')
            data = load('./utils/toolbox_map.mat');
            keys = data.keys;
            values = data.values;
        else
            keys = {};
            values = {};
        end

        % 기존 Map
        nameToId = containers.Map(keys, values);

        % 현재 설치된 툴박스
        installed = matlab.addons.installedAddons;
        currentNames = cellstr(installed.Name);
        currentIds   = cellstr(installed.Identifier);

        % 새로운 항목 추가
        count = 0;
        for i = 1:length(currentNames)
            name = currentNames{i};
            id = currentIds{i};
            if ~isKey(nameToId, name)
                nameToId(name) = id;
                count = count + 1;
            end
        end

        % 저장
        keys = nameToId.keys;
        values = nameToId.values;
        save('./utils/toolbox_map.mat', 'keys', 'values');
    else
        data = load('./utils/toolbox_map.mat');
        nameToId = containers.Map(data.keys, data.values);
    
        installed = matlab.addons.installedAddons;
        installedNames = string(installed.Name);
    
        flag = false;
        for i = 1:length(required)
            rname = required(i);
            if ~any(installedNames == rname)
                if isKey(nameToId, char(rname))
                    rid = nameToId(char(rname));
                    link = sprintf( ...
                        "<a href=""matlab:matlab.internal.addons.launchers.showExplorer('ErrorRecovery', 'identifier', '%s', 'focused', 'addon')"">%s</a>\n", ...
                        rid, rname);
                    fprintf("Missing toolbox: %s", link);
                else
                    fprintf("Missing toolbox: %s\n", rname);
                end
                flag = true;
            end
        end
    
        if flag
            error("Missing toolbox(es).");
        end
    end

end
