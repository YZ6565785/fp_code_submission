%%
clear;
close all;

%% iiwa14
save_IRM = false;
load_IRM = true;

robot_path = 'iiwa_description/robots/iiwa14.urdf';

% set boolean indices file
fixed_joint = 2
r = 0.08
% set boolean indices file
file_rm_bl = "bl_iiwa14_r"+r+"_reachability_50poses.txt";
path_rm_bl = "Victor/" + file_rm_bl;

parts = split(file_rm_bl, "_");
robot_name = parts(2)
res = str2double(extractAfter(parts(3),2));

%% LOAD BOOLEAN INDICES
disp("Hold on. Loading boolean indices...");
fileID = fopen(path_rm_bl,'r');
formatSpec = '%c';
all = fscanf(fileID,formatSpec);
lines = splitlines(all);
fclose(fileID);
disp("Still loading...");

progress = 1;
check = round(size(lines,1)/10);
bool_indices = [];
for i=1:size(lines)
    if mod(progress, check)== 0
        disp("Progress: " + round(progress/size(lines,1)*100) + "%");
    end
    fields = split(lines(i), ',');
    if str2num(fields{1}) > 0
       bool_indices = [
           bool_indices; 
           str2num(fields{1}) str2num(fields{2}) str2num(fields{3})
       ];
    end
    progress = progress + 1;
end
num_poses = size(bool_indices,2)-4; 

disp("Done!");
disp("Resolution: " + res);
disp("Samplings per sphere: " + num_poses);
disp("Total: " + size(bool_indices,1) + " poses.");

%% FILTER OUT EMPTY SPHERES FOR BETTER VISUALIZATION
indices = ~(bool_indices(:,2)<0.2 & bool_indices(:,4)>0.1);
H = bool_indices(indices,1);
sphere_X = bool_indices(indices,2);
sphere_Y = bool_indices(indices,3);
sphere_Z = bool_indices(indices,4);
disp("*** Filtered.");


% CONVERT SCALAR TO COLOR (rgb) - scaling from red to blue (0-100)

sphere_color = PlotTools.generateRGB(H, max(H));

% PLOT robot and RM
figure(1);
clf
robot = importrobot(robot_path);
show(robot, 'Collisions','off','Visuals','on');
hold on
scatter3(sphere_X', sphere_Y', sphere_Z', 90, sphere_color, 'filled');
title('Reachability Map and Robot');
colorbar
axis equal


%% PLOT CONSTRAINED RM AND SAMPLE POSES - ZACHARIAS SAMPLING
samples = InverseReachability.zacharias(res, num_poses);

mask = PlotTools.generateMask(samples, 3, true);

bl = bool_indices(:,5:end) & mask';
ri = sum(bl,2);
indices = ~(bool_indices(:,2)<0.2 & bool_indices(:,4)>0.1) &...
    ri>0;
H = ri(indices);
sphere_X = bool_indices(indices,2);
sphere_Y = bool_indices(indices,3);
sphere_Z = bool_indices(indices,4);

sphere_color = PlotTools.generateRGB(H, max(H));

figure(1);
clf;

% Top plot
subplot(2,1,1)
robot = importrobot(robot_path);
show(robot, 'Collisions','off','Visuals','on');
hold on;
scatter3(sphere_X', sphere_Y', sphere_Z', 50, sphere_color, 'filled');

title('Reachability Map and Robot with Mask');
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')

% plot local sphere poses with mask
% Bottom plot
subplot(2,1,2)
[X,Y,Z] = sphere;


r = res;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
hold on
surf(X2,Y2,Z2)
axis equal
Visualization.draw_poses(samples(mask,:), 0.5, [1 0 0], 'r');
Visualization.draw_poses(samples(mask,:), 0.5, [0 1 0], 'g');
Visualization.draw_poses(samples(mask,:), 0.5, [0 0 1], 'b');
hold off

xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
title('Local Sphere Poses with Mask')

%% IRM - Complete
if save_IRM
    disp("Computing IRM");
    base_z = 0;
    r = 1; % length that arm reaches
    res_IRM = res;
    deg = 9;
    th_interval = deg/180*pi;

    z_itr = 0.1;
    progress = 1;
    total = (2*pi-0.1)/th_interval*(2*r/res_IRM)^2*(2*r/z_itr);
    check = round(total/10);


    irm = IRMTools.IRM();
    for z=-r:z_itr:r % z coord for task
        bl_IRM = [];
        bl_IRM_sum = [];
        for h=-r:res_IRM:r
            for w = -r:res_IRM:r
                bl_sum = 0;
                x = w; % x coord for base placement
                y = h; % y coord for base placement

                for th=0:th_interval:2*pi-th_interval

                    R = Helpers.rot2d(th);
                    coord = inv(R)*[x;y];
                    ind = find(abs(bool_indices(:,2) - coord(1))<=0.025 &...
                        abs(bool_indices(:,3) - coord(2))<=0.025 &...
                        abs(bool_indices(:,4) - z)<=0.025, 1);

                    if ~isempty(ind)
                        bl = bool_indices(ind,5:end);
                        bl_IRM = [bl_IRM; sum(bl) x y base_z th bl];
                        bl_sum = bl_sum + sum(bl);
                    else
        %                 disp("not found in RM");
                    end
                    progress = progress + 1;
                    if mod(progress, check)== 0
                        disp("Progress: " + round(progress/total*100) + "%");
                    end
                end
                bl_IRM_sum = [bl_IRM_sum; bl_sum x y base_z];
            end

        end
        % store IRM
        s = struct;
        s.z = num2str(z);
        s.bl = bl_IRM;
        s.bl_sum = bl_IRM_sum;
        irm.map = [irm.map; s];
    end
    % size(bl_IRM)       
    % size(bl_IRM_sum)
    disp("Done!");
    disp("IRM Resolution: " + res_IRM);

    % save IRM !!!!!
    IRM_file_path = "Victor/"+"IRM_"+robot_name+"_r"+res_IRM+"_"+num_poses+"poses"+datestr(now,30)+".mat";
    save(IRM_file_path, 'irm');
    disp("IRM saved to: " + IRM_file_path);
end


%% load IRM !!!!!

if load_IRM
    IRM_file_name = "";
    files = dir("Victor");
    file_names = {files.name};
    sort(file_names)
    for i=length(file_names):-1:1
       if contains(file_names{i},"IRM_"+robot_name)
           IRM_file_name = file_names{i};
           break;
       end
    end
    if IRM_file_name == ""
        disp("No IRM for "+robot_name+" saved before.");
    else
        disp(IRM_file_name)
        load("Victor/"+IRM_file_name);
        disp("*** IRM loaded with " + size(irm.map,1) + " maps.");
    end
end
%% FILTER OUT EMPTY base voxels FOR BETTER VISUALIZATION
map = findMap(irm, 0.7);
disp("found " + size(map.bl,1) + " poses in this IRM at z="+map.z);
if size(map.bl,1) ==0
    disp("So, not showing any.");
    figure(2); clf; title("No IRM available");
else
    bl_IRM_sum = map.bl_sum;
    indices = bl_IRM_sum(:,1)>0;

    H = bl_IRM_sum(indices,1);
    base_X = bl_IRM_sum(indices,2);
    base_Y = bl_IRM_sum(indices,3);
    base_Z = bl_IRM_sum(indices,4);

    base_color = PlotTools.generateRGB(H, max(H));
    figure(2);
    scatter3(base_X, base_Y, base_Z, 90, base_color, 'filled');
    colorbar
    axis equal
    title("IRM at z="+map.z);
end

%%
z_list = [0 0.2 0.4 0.6 0.8];
subplot(2,1,2)
for i=1:size(z_list)
   map = findMap(irm, z_list(i));
    disp("found " + size(map.bl,1) + " poses in this IRM at z="+map.z);
    if size(map.bl,1) ==0
        disp("So, not showing any.");
        
    else
        bl_IRM_sum = map.bl_sum;
        indices = bl_IRM_sum(:,1)>0;

        H = bl_IRM_sum(indices,1);
        base_X = bl_IRM_sum(indices,2);
        base_Y = bl_IRM_sum(indices,3);
        base_Z = bl_IRM_sum(indices,4);

        base_color = PlotTools.generateRGB(H, max(H));
        
        scatter3(base_X, base_Y, base_Z, 90, base_color, 'filled');
        colorbar
        axis equal
        title("IRM at z="+map.z);
    end 
end
