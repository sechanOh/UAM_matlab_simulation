% OBJ 파일 경로
filename = "C:/Users/SechanOh/Downloads/uploads_files_2336673_Fire.obj";

% OBJ 파일 열기
fid = fopen(filename, 'r');
if fid == -1
    error('파일을 열 수 없습니다.');
end

vertices = [];
faces = [];

% 줄 단위 파싱
while ~feof(fid)
    tline = strtrim(fgetl(fid));
    
    if startsWith(tline, 'v ')
        % vertex line
        nums = sscanf(tline(3:end), '%f %f %f');
        vertices(end+1, :) = nums';
        
    elseif startsWith(tline, 'f ')
        % face line: 형식은 보통 f v1 v2 v3 또는 f v1//vn1 ...
        % 공백 기준으로 분리
        tokens = split(tline);
        indices = zeros(1, 3);
        for i = 2:4  % f 다음 세 개
            part = split(tokens{i}, '/');
            indices(i-1) = str2double(part{1});
        end
        faces(end+1, :) = indices;
    end
end

fclose(fid);

% 시각화
figure;
trisurf(faces, vertices(:,1), vertices(:,2), vertices(:,3), ...
        'FaceColor', [1 0.4 0], 'EdgeColor', 'none');
axis equal;
view(3);
camlight;
lighting gouraud;
title('OBJ Mesh without texture');
xlabel('X'); ylabel('Y'); zlabel('Z');
