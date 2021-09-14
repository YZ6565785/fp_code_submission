%% import
clear all;
close all;
addpath(genpath('Victor'))
addpath(genpath('TCPP'))

%%
th = pi;
rad = 0.8;
sft = 0.1
t = linspace(0, 1, 100);

%%
x = cos(th*t) * rad;
y = zeros(size(t)) * rad;
z = sin(th*t)*0.5 * rad;

figure(1);clf;hold on
scatter3(x, y, z)
axis equal
axis([-1 1 -1 1 -1 1])
p = [x; y; z]'
normalise_path(p)
scatter3(p(:,1),p(:,2),p(:,3),'+')
%%
pipe_x = 0.5;
pipe_rad = [0.3 0.5];
task = TaskTools.PipePath(pipe_x, pipe_rad);

points = task.path;
task_pose_vec = TForm.tform2vec(task.T);
% plot
f=figure(1);clf;
scatter3(points(:,1),points(:,2),points(:,3),'filled')
PlotTools.plotPoses(task_pose_vec,0.3)
axis equal
xlabel("x")
ylabel("y")
zlabel("z")
axis([-0.2 1.1 -0.25 0.25 0 0.6])
view([-15 5])
if false
   saveas(f, "../UCL Final Project/figures/task/task_poses.png", 'png') 
end

% dont below show for now
% task_pose_vec = TForm.tform2vec(task.getT());
% 
% figure(2);clf;
% scatter3(points(:,1),points(:,2),points(:,3),'filled')
% PlotTools.plotPoses(task_pose_vec,0.1)
% axis equal
% axis([-1 1 -1 1 -1 1])
% 
% figure(3);clf;
% scatter3(points(:,1),points(:,2),points(:,3),'filled')
% axis equal
% axis([-1 1 -1 1 -1 1])
% 
% points = task.points;
% task_pose_vec = TForm.tform2vec(task.pointsToTForm());
% 
% figure(4);clf
% scatter3(points(:,1),points(:,2),points(:,3),'o')
% PlotTools.plotPoses(task_pose_vec,0.1)
% axis equal
% axis([-1 1 -1 1 -1 1])
%% test U pattern
q = Tasks.UPath(.2)
points = q.path;

figure(1);clf;
scatter3(points(:,1),points(:,2),points(:,3),'filled')

%%
q_T = q.toTForm(q);
q_vec = TForm.tform2vec(q_T);

figure(1);clf;
PlotTools.plotPoses(q_vec, 0.1)
axis([-0.4 1.2 -1.2 1.2 -0.4 1.2])
%%  arched around z .:'`':.
task = TaskTools.Task(pi);

points = task.path;
vec = TForm.tform2vec(task.toTForm(task));
% plot
figure(1);clf;hold on
scatter3(points(:,1),points(:,2),points(:,3),'filled')
PlotTools.plotPoses(vec, 0.1)

axis equal
%%  arched around y .:'`':.
task = TaskTools.PipePath(0.5, [0.1 0.1]);

points = task.path;
vec = TForm.tform2vec(task.T);
% plot
figure(1);clf;hold on
scatter3(points(:,1),points(:,2),points(:,3),'filled')
axis equal
axis([-1 1 -1 1 -1 1])
xlabel('x')
ylabel('y')
zlabel('z')

figure(2);clf;hold on
PlotTools.plotPoses(vec, 0.1)

axis equal
axis([0 1 -1 1 -0.5 0.5])
xlabel('x')
ylabel('y')
zlabel('z')


vec = TForm.tform2vec(task.pointsToTForm());

figure(3);clf;hold on
PlotTools.plotPoses(vec, 0.1)

axis equal
axis([0 1 -1 1 -1 1])
xlabel('x')
ylabel('y')
zlabel('z')
%% toTForm2 - to make the transformation matrices correct

points_T = task.pointsToTForm();
n_points = size(task.points,1)

path_ = task.path;
n = size(path_,1);
ax = diff(path_-1,1);
ax = [ax; ax(end,:)];
axx = path_ - path_(1,:);
tform = zeros(4, 4, n);
for i = 1:n
    ind = min(n_points, ceil(i/n*n_points));
    if path_(i,1) > 0 % a little trick
        ind = find( abs(points_T(1,4,:)-path_(i,1))<0.005, 1,'last')
    else
        ind = find( abs(points_T(1,4,:)-path_(i,1))<0.005, 1)
    end
    rotm = tform2eul(points_T(:,:,ind));
    
    tform(:,:,i) = points_T(:,:,ind);
    tform(1:3, 4, i) = path_(i, :)';
end


vec = TForm.tform2vec(tform);

figure(3);clf;hold on
axis equal
axis([-1 1 -1 1 -1 1])

PlotTools.plotPoses(vec, 0.2)


%%
figure(2);clf
scatter3(task.path(:,1),task.path(:,2),task.path(:,3))
axis equal
%%
function path_ = normalise_path(path_)
    % normalise  given path so that its amplitude in any direction is max 1.
    normalise = @(x) x ./ max(abs(x));
    path_ = [normalise(path_(:, 1)), normalise(path_(:, 2)), normalise(path_(:, 3))];
    path_(isnan(path_)) = 0;
end