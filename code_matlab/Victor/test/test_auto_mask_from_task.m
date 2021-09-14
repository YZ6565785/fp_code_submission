
%% load Robot and IRM

ROBOT_MODEL = 'xarm6_box_base2_ee';
IRM_RES = 0.05;

irm = IRMTools.loadIRM(IRM_RES, ROBOT_MODEL);

% Define Task
task = TaskTools.PipePath(1.6)
figure(1);clf;hold on
task.plot();
PlotTools.plotPoses(TForm.tform2vec(task.T), 0.1)
axis equal
axis([-1.5 1.5 -1 1 -1 1])


% add world obstacles
box_T = eye(4);

box_l = 0.3;
positions = [0.5 -0.3 box_l; 1 -0.25 box_l; 1.4 0.2 box_l; 1 0.75 box_l; -0.5 0.5 box_l];
n = length(positions);
world_objects = cell(1, n);

for i=1:n
    box = collisionBox(0.3, 0.3, 0.3);
    box_T(1:3,4) = positions(i,:)';
    if i==4
        box_T(1:2,1:2) = Helpers.rot2d(pi/4);
    else
        box_T(1:2,1:2) = eye(2);
    end
    box.Pose = box_T;
    world_objects{i} = box;
end

hold on
for i=1:length(world_objects)
    show(world_objects{i});
end
hold off
xlabel('x')
ylabel('y')
zlabel('z')

%% plot masks for task

th = pi*60/180;
T_turn = eul2tform([0 pi 0], 'ZYZ')


figure(1);clf;hold on
task.plot()
for j=1:size(task.path,1)
    
    u = task.T(1:3,1:3,j) * T_turn(1:3,1:3) * [0;0;1];
    
    S = irm.getSampledPoses();
    mask = false(size(S,1),1);
    for i=1:size(S,1)
       v = S(i,1:3)';
       mask(i,1) = (atan2(norm(cross(u,v)), dot(u,v)) <= th);
    end
    
    S = S(mask,:) + [task.T(1:3,4,j)' 0 0 0 0];
    
    PlotTools.plotPoses(S,0.1);
    
end
axis([-1 1 -1 1 -1 1])


axis equal
