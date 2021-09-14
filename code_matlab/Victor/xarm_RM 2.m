%%
clear;
close all;
%% 

%set robot path
robot_path = "xarm_description/robots/xarm6_robot.urdf";

% set boolean indices file
file_rm_bl = 'Victor/bl_xarm6_r0.05_reachability_200_poses.txt';

% set hdf5 file
file = "xarm6_r0.05_reachability_200_poses.h5";
path = "Victor/" + file;

num_poses = 200

% check H5 info
h5disp(path);

info = h5info(path)

% LOAD DATASET FROM H5
pose_dataset = h5read(path, '/Poses/poses_dataset');
sphere_dataset = h5read(path, '/Spheres/sphere_dataset');
res = info.Groups(2).Datasets(1).Attributes(1).Value;
disp('# Summary:');
disp("pose dataset size: "+ size(pose_dataset,1)+"x"+ ...
    size(pose_dataset,2));
disp("sphere dataset size: "+ size(sphere_dataset,1)+"x"+ ...
    size(sphere_dataset,2));
disp("Resolution: " + res);

% LOAD BOOLEAN INDICES
disp("Hold on. Loading boolean indices...");
fileID = fopen(file_rm_bl,'r');
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
disp("Done!");
disp("Total: " + size(bool_indices,1));

%% FILTER OUT EMPTY SPHERES FOR BETTER VISUALIZATION
indices = ~(bool_indices(:,2)<0.2 & bool_indices(:,4)>0.1);
H = bool_indices(indices,1);
sphere_X = bool_indices(indices,2);
sphere_Y = bool_indices(indices,3);
sphere_Z = bool_indices(indices,4);
disp("*** Filtered.");

%%%
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
mask = PlotTools.generateMask(samples, 2, false);

% Filter
bl = bool_indices(:,5:end) & mask';
ri = sum(bl,2);
indices = ~(bool_indices(:,2)<0.2 & bool_indices(:,4)>0.1) &...
    ri>0;
H = ri(indices);
sphere_X = bool_indices(indices,2);
sphere_Y = bool_indices(indices,3);
sphere_Z = bool_indices(indices,4);

sphere_color = PlotTools.generateRGB(H, max(H));

figure(2);
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
figure(1);
clf
disp("Computing IRM");
base_z = 0;
r = 1; % length that arm reaches
res_IRM = 0.05
th_interval = 9/180*pi;

progress = 1;
total = (2*pi-0.1)/th_interval*(2*r/res_IRM)^2;
check = round(total/10);

bl_IRM = [];
bl_IRM_sum = [];
X2d = [];
Y2d = [];
for h=-r:res_IRM:r
    for w = -r:res_IRM:r
        bl_sum = 0;
        x = w; % x coord for base placement
        y = h; % y coord for base placement
        
        z = 0; % z coord for task
        for th=0:th_interval:2*pi-0.1
            if mod(progress, check)== 0
                disp("Progress: " + round(progress/total*100) + "%");
            end
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
        end
        bl_IRM_sum = [bl_IRM_sum; bl_sum x y base_z];
    end
end
size(bl_IRM)
size(bl_IRM_sum)

% FILTER OUT EMPTY base voxels FOR BETTER VISUALIZATION
indices = bl_IRM_sum(:,1)>0;
H = bl_IRM_sum(indices,1);
base_X = bl_IRM_sum(indices,2);
base_Y = bl_IRM_sum(indices,3);
base_Z = bl_IRM_sum(indices,4);

base_color = PlotTools.generateRGB(H, max(H));
figure(1);
scatter3(base_X, base_Y, base_Z, 90, base_color, 'filled');
colorbar
axis equal