function [bool_indices, num_poses] = loadBl(bl_file_path, res)
    % LOAD BOOLEAN INDICES
    disp("Hold on. Loading boolean indices...");
    fileID = fopen(bl_file_path,'r');
    formatSpec = '%c';
    all_lines = fscanf(fileID,formatSpec);
    lines = splitlines(all_lines);
    clear all_lines;
    fclose(fileID);
    disp("Still loading...");

    progress = 1;
    check = round(size(lines,1)/100);
%     bool_indices = [];
    n = size(lines, 1);
    d = size(str2num(lines{1}), 2);
    bool_indices = [];
    for i=1:n

        if size(lines{i},2)~=0
            bl = str2num(lines{i});
            if bl(1)~=0
                bool_indices = [bool_indices; bl];
            end
        end

        if mod(progress, check)== 0 || progress == n
            disp("Progress: " + round(progress/n*100) + "%");
        end
        
        progress = progress + 1;
    end

    num_poses = d-4; 

    clear lines;
    disp("Done!");
    disp("Resolution: " + res);
    disp("Samplings per sphere: " + num_poses);
    disp("Total: " + size(bool_indices,1) + " poses.");
end