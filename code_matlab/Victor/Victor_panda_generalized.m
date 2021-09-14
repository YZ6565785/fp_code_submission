%%
robot = loadrobot('frankaEmikaPanda');
eeOffset = [0 0 0.1];
%%
showdetails(robot);

%%
% create ik solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = ones(1,6); % only x and y coordinates are important
endEffector = 'panda_hand';

%%
homePose = TForm.DOWN;
homePose(1:3,4) = [0.5, 0, 0];

q0 = homeConfiguration(robot);
[q0Sol, solInfo]= ik(endEffector,homePose,weights,q0);
solInfo
show(robot, q0Sol);

%%
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];
points

%%
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
solutions = [];

%%
eeOrientation = [0, pi, 0];
%%
valid = zeros(1,size(points,1));
% loop through the trajectory points
qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    targetPose = eul2tform(eeOrientation);
    targetPose(1:3,4) = point + eeOffset;
%     trvec2tform(point)
    [qSol, solInfo] = ik(endEffector,targetPose,weights,qInitial);
    valid(i) = solInfo.ExitFlag==1;
%     disp([qSol.JointPosition]);
    solutions = [solutions; qSol];
    % Store the configuration
    qs(i,:) = [qSol.JointPosition];
    % Start from prior solution
    qInitial = qSol;
end

%%
figure
show(robot,solutions(1,:));
% show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])













