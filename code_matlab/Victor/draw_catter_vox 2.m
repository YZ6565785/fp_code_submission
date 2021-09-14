

%%
clear all;
load victor_0.3rad_grid_complete2_ubuntu.mat


%% for iiwa14 robot
robot = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';
show(robot, 'Collisions','on','Visuals','off');

showdetails(robot);
ee = "iiwa_link_ee_kuka";

%%
vox.xc = zeros(size(RM.voxCenters,1),1);
vox.yc = zeros(size(RM.voxCenters,1),1);
vox.zc = zeros(size(RM.voxCenters,1),1);
for i=1:size(RM.voxCenters,1)
    vox.xc(i) = RM.voxCenters(i,1);
    vox.yc(i) = RM.voxCenters(i,2);
    vox.zc(i) = RM.voxCenters(i,3);
end

vox.N = ones(size(RM.voxCenters,1),1)*RM.params.gridres;


%%
figure(1);
hold on;
X = vox.xc;
Y = vox.yc;
Z = vox.zc;
N = vox.N;

% reference to scatter_vox
scatter3(X(N>0),Y(N>0),Z(N>0),20);


%% draw local sphere

X = zeros(size(RM.LocalRIPoses,3), 1);
Y = zeros(size(RM.LocalRIPoses,3), 1);
Z = zeros(size(RM.LocalRIPoses,3), 1);
for i=1:size(RM.LocalRIPoses,3)
    X(i) = RM.LocalRIPoses(1,4,i); 
    Y(i) = RM.LocalRIPoses(2,4,i); 
    Z(i) = RM.LocalRIPoses(3,4,i); 
end
%%
figure
scatter3(X,Y,Z)

%%
n = 10;                             % Facets On Sphere
[Xs,Ys,Zs] = sphere(n);
[Nx,Ny,Nz] = surfnorm(Xs,Ys,Zs);
% Ns = reshape([Nx,Ny,Nz], n+1, n+1, 3);
figure(1)
surf(Xs,Ys,Zs)
hold on
% plot3(Ns(:,:,1), Ns(:,:,2), Ns(:,:,3))
quiver3(Xs,Ys,Zs, Nx,Ny,Nz)
hold off
grid on
axis equal


%% %%%%%% Voxelise into IRM %%%%%%
poses = ik_result.ik_exists_vec;
RM = TForm.vec2tform(poses);
IRM = TForm.tform2inv(RM);
iposes = TForm.tform2vec(IRM);

%Voxelise
VoxIRM = voxelise(iposes, params.samplingbias_vox_size, horzcat(ik_result.sols_found{:})'); %xyzrpy
VoxRM = voxelise(poses, params.samplingbias_vox_size, horzcat(ik_result.sols_found{:})'); %xyzrpy
experiment.VoxIRM = VoxIRM;
experiment.VoxRM = VoxRM;
save(params.trial_name + "_exp", "experiment");







