%%
close all
clear all
addpath(genpath('Victor'))
addpath(genpath('TCPP'))
%%
save_IRM = false; % compute and save IRM?
load_IRM = true; % load old IRM?

robot_name = "xarm6_box_base2_ee";
res = 0.05;
num_pose = 100;
IRM_path = "Victor/data/irm_saved/";

bl_path = "../reuleaux_fp/map_creator/maps/";
bl_file_name = 'bl_'+robot_name+'_r'+res+'_reachability_'+num_pose+'poses.txt';
bl_file_path = bl_path + bl_file_name;
assert(isfile(bl_file_path), bl_file_path + newline + 'is not a valid boolean file');
%set robot path
robot_path = "../xarm_fp/xarm_description/robots/"+robot_name+".urdf"
assert(isfile(robot_path), robot_path + newline + 'is not a valid robot path');

% [res, robot_name] = RMTools.getResAndRobotName(bl_file_name)
res_IRM = res
%% LOAD BOOLEAN INDICES
[bool_indices, num_poses] = RMTools.loadBl(bl_file_path, res);

%%
% FILTER OUT EMPTY SPHERES FOR BETTER VISUALIZATION
indices = bool_indices(:,2)>=0.5 | bool_indices(:,3)>=0.1 | bool_indices(:,4)<=0;
H = bool_indices(indices,1);
sphere_X = bool_indices(indices,2);
sphere_Y = bool_indices(indices,3);
sphere_Z = bool_indices(indices,4);
disp("*** Filtered.");

% CONVERT SCALAR TO COLOR (rgb) - scaling from red to blue (0-100)
figure(1);
clf;
PlotTools.plotRM(robot_path, sphere_X, sphere_Y, sphere_Z, H)

%% PLOT CONSTRAINED RM AND SAMPLE POSES - ZACHARIAS SAMPLING
samples = InverseReachability.zacharias(res, num_poses);
mask = PlotTools.generateMask(samples, 1, false,45);

f = figure(2);
clf;
method = 2;
PlotTools.plotRMWithMask(robot_path, bool_indices, samples, mask, res, [-20,15], method);

if method == 1 
    image_name = "RM_global_z_pos_direction";
    save_confirmed = input("save the file to "+image_name+"? [y/Any]: ", 's');
    if save_confirmed == 'y'
        saveas(f, "../../../../../../Downloads/UCL Final Project/figures/imp/"+image_name+".png", 'png')
        disp('Saved.')
    end
end
%%
th = [65 55 45 35 25 15]
f = figure(3);clf;
for i=1:size(th,2)
    
    mask = PlotTools.generateMask(samples, 3, true,th(i));

    method = 2;
    subplot(2,size(th,2)/2,i)
    PlotTools.plotRMWithMask(robot_path, bool_indices, samples, mask, res, [-20, 15], method);
    title("Constrainted RM with mask(th="+th(i)+")")
    
end
if false
    saveas(f, "../UCL Final Project/figures/imp/RM_z_pos_direction_th=65-15.png", 'png')
end
%% IRM - test
if false
    r = 1 % length that arm reaches
    res_IRM = 0.05

    X2d = [];
    Y2d = [];
    bl_IRM = []
    x = 0.5;
    y = 0;
    z = 0;
    for th=0:(9/180*pi):2*pi-0.1
        R = Helpers.rot2d(th);
        coord = inv(R)*[x;y];
        ind = find(abs(bool_indices(:,2) - coord(1))<=0.025 &...
            abs(bool_indices(:,3) - coord(2))<=0.025 &...
            abs(bool_indices(:,4) - z)<=0.025, 1);

        bl = bool_indices(ind,5:end);
        bl_IRM = [bl_IRM; sum(bl) x y z th bl];
        X2d = [X2d; coord(1)];
        Y2d = [Y2d; coord(2)];
    end
    figure(1);
    clf
    scatter(X2d, Y2d);
    axis equal
end

% bl_IRM(:,1:5);

%% IRM - Complete
if save_IRM
    irm = IRMTools.computeIRM(bool_indices, res_IRM);
    irm.robot_name = robot_name;
    % save IRM !!!!!
    IRM_file_path = IRM_path +...
        "IRM_"+irm.robot_name+"_r"+irm.res+"_"+irm.num_poses+"poses"+datestr(now,30)+".mat";
    save(IRM_file_path, 'irm');
    disp("IRM saved to: " + IRM_file_path);
end

%% load IRM !!!!!

if load_IRM
    IRM_file_name = "";
    files = dir(IRM_path);
    file_names = {files.name};
    sort(file_names)
    for i=length(file_names):-1:1
       if contains(file_names{i},"IRM_"+robot_name+"_r"+res_IRM+"_"+num_poses+"poses")
           IRM_file_name = file_names{i};
           break;
       end
    end
    if IRM_file_name == ""
        disp("No IRM for "+robot_name+" saved before.");
    else
        disp(IRM_file_name)
        load(IRM_path+IRM_file_name);
        disp("*** IRM loaded with " + size(irm.map,1) + " maps.");
    end
end

%% FILTER OUT EMPTY base voxels FOR BETTER VISUALIZATION
f = figure(3);
clf;
irm_layer = findMap(irm, 0);
PlotTools.plotIRMLayer(irm_layer);
view([0 90])
axis([-1 1 -1 1 -1 1])
title("IRM Layer Task z=0")
xlabel("x")
ylabel("y")
% saveas(f, "../../../../../../Downloads/UCL Final Project/figures/imp/IRM_z=0.png", 'png')

%%
figure(4);clf;

z_list = -0.2:0.1:1;

n = size(z_list,2);
n_col = 5;
n_row = ceil(n/n_col);

inds_x = [1:n_col]-1;
X = inds_x*1/n_col;
inds_y = [1:n_row]-1;
Y = inds_y*1/n_row;
for i=1:n_row
    for j=1:n_col
        num = (i-1)*n_col+j;
        if num<=n
            disp("="+num+" "+"("+(X(j)+0.05)+","+(Y(i)+0.1)+")");
            fig = subplot(n, n_col, num);
            set(fig, 'Position', [X(j)+0.05, Y(i)+0.1, ...
                0.7/n_col 0.6/n_row]);
            irm_layer = findMap(irm, z_list(num));
            PlotTools.plotIRMLayer(irm_layer);
            hold on
        end
    end

end


%% Sampling IRM from task






























%% genreate points on sphere - REULEAUX
if false
origin = [0 0 0];

pose_Col = make_sphere_poses(origin, res);
disp("Genreated poses.");


% test reuleaux's sampling
if false
figure(3)
clf
[X,Y,Z] = sphere;
axis equal

hold on
r = res;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

surf(X2,Y2,Z2)
axis equal

sample1 = make_sphere_poses(origin, res);

X = sample1(:,1);
Y = sample1(:,2);
Z = sample1(:,3);
hold on
Visualization.draw_poses(sample1, 0.5, [1 0 0], 'r');
Visualization.draw_poses(sample1, 0.5, [0 1 0], 'g');
Visualization.draw_poses(sample1, 0.5, [0 0 1], 'b');
hold off

% [xGrid,yGrid] = meshgrid(linspace(min(X),max(X)),linspace(min(Y),max(Y)));
% zGrid = griddata(X(:),Y(:),Z(:),xGrid(:),yGrid(:),'cubic');
% zGrid = reshape(zGrid,size(xGrid));
% 
% [Nx,Ny,Nz] = surfnorm(X,Y,Z)
% surf(X,Y,Z)
% scatter3(X,Y,Z, 'filled');
axis equal
title('reuleaux sampling poses')
end

%
x = linspace(0,10,50);
y1 = sin(x);
y2 = rand(50,1);
tiledlayout(2,1) % Requires R2019b or later

% Top plot
nexttile
plot(x,y1)
title('Plot 1')

% Bottom plot
nexttile
scatter(x,y2)
title('Plot 2')
end
%%
function [pose_Col] = make_sphere_poses(origin, r)
    DELTA = pi/5;
    MAX_INDEX = (2 * 5 * 5);
    position_vector = zeros(MAX_INDEX, 3);
    quaternion = zeros(MAX_INDEX, 4);
    initialized = false;
    if ~initialized
        initialized = true;
        index = 1;
        phi = 0;
        while phi < 2*pi
            theta = 0;
            while theta < pi
                position_vector(index, 1) = cos(phi) * sin(theta);
                position_vector(index, 2) = sin(phi) * sin(theta);
                position_vector(index, 3) = cos(theta);
                quaternion(index,:) = angle2quat(0, ((pi/2)+theta), phi, 'XYZ');

                theta = theta + DELTA; 
                index = index + 1;
            end
            phi = phi + DELTA;
        end
    end

    pose_Col = [];
    for i=1:MAX_INDEX
        x = r * position_vector(i, 1) + origin(1);
        y = r * position_vector(i, 2) + origin(2);
        z = r * position_vector(i, 3) + origin(3);
        qw = quaternion(i, 1);
        qx = quaternion(i, 2);
        qy = quaternion(i, 3);
        qz = quaternion(i, 4);
        pose_Col = [pose_Col; x y z qx qy qz qw];
    end
end


