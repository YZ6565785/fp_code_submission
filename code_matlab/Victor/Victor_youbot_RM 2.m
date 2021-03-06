%% for youbot
robot = importrobot("youbot-ros-pkg-kdl-master/robots/youbot.urdf");
robot.DataFormat = 'column';

% remove the collision parts
% four wheels, and the laser sensor at the front
clearCollision(robot.Bodies{13});
clearCollision(robot.Bodies{15});
clearCollision(robot.Bodies{17});
clearCollision(robot.Bodies{19});
for i=11:11 
    clearCollision(robot.Bodies{i});
end


show(robot, 'Collisions','on','Visuals','off');
showdetails(robot);
ee = "gripper_palm_link";

isColliding = checkCollision(robot,homeConfiguration(robot));
printCollision(isColliding);

%% set a non-collision pose
ik = inverseKinematics("RigidBodyTree", robot);
weights = ones(1,6);


q0 = homeConfiguration(robot);
q0

pose1 = TForm.DOWN;
    pose1(1:3, 4) = [0.4, 0, 0.];
[poseSol, poseInfo] = ik(ee, pose1, weights, q0);
poseSol
poseInfo

sol = poseSol;
show(robot, sol, 'Collisions','on','Visuals','off');
isColliding = checkCollision(robot,sol);
printCollision(isColliding);


%% set a collision pose
ik = inverseKinematics("RigidBodyTree", robot);
weights = ones(1,6);


q0 = homeConfiguration(robot);
q0

pose1 = TForm.DOWN;
    pose1(1:3, 4) = [0.2, 0, 0.17];
[poseSol, poseInfo] = ik(ee, pose1, weights, q0);
poseSol
poseInfo

sol = poseSol;
show(robot, sol, 'Collisions','on','Visuals','off');
isColliding = checkCollision(robot,sol);
printCollision(isColliding);


%% set up Params
% params.gridrad = 1.5;
params.gridrad = 0.3;
params.gridres = 0.05;
params.sample_n = 50;


params.trial_name = "victor_youbot_0.3rad_grid";
params.trial_description = "Try small grid first.";
RM.params = params;

%% create grid
idx_rad = ceil(params.gridrad ./ params.gridres);
d = (-idx_rad:idx_rad) * params.gridres;
M = length(d);
MM = M^3;
[X, Y, Z] = meshgrid(d, d, d);
RM.spans = {d, d, d};
RM.grid.X = X;
RM.grid.Y = Y;
RM.grid.Z = Z;

%% create voxels
RM.voxCenters = [X(:), Y(:), Z(:)];
RM.voxRIPoses = cell(M^3, 1);
RM.voxIKSuccess = cell(M^3, 1);
RM.voxIKSolutions = cell(M^3, 1);
RM.voxValid = false(MM, 1);
RM.voxRI = zeros(MM,1);
RM.voxRICenterSol = cell(MM,1);
RM.voxCenterSolInfo = cell(MM,1);


%% set up joint limits, 5 joints for youbot
ups = zeros(1,5);
downs = zeros(1,5);
for i=1:5
    limits = robot.Bodies{i+2}.Joint.PositionLimits;
    downs(i) = limits(1);
    ups(i) = limits(2);
end

%% compute ik solutions for voxel centers
POSE = repmat(TForm.DOWN, 1, 1);
check = round(MM/100);
for i=1:MM
    if mod(i,check) == 0
        disp(string(i*100/MM) + '%');
    end
    POSE(1:3, 4) = RM.voxCenters(i,:);
%     POSE
    [qSol, solInfo]= ik(ee, POSE, weights, q0);
    
%     for j=1:5
%         if (qSol(j) > ups(j) || qSol(j) < downs(j))
%             
%             disp(downs(j) + ", " + ups(j));
%             disp("invalid solution, out of joint limits! " + string(i));
%             disp(qSol(j));
%             RM.voxValid(i) = false;
%         else
%             RM.voxValid(i) = solInfo.ExitFlag==1;
%         end
%     end
    RM.voxValid(i) = solInfo.ExitFlag==1;
    RM.voxRICenterSol{i} = qSol;
    RM.voxCenterSolInfo{i} = solInfo;
%     q0 = qSol;
%     show(robot, qSol);
end

save(params.trial_name+'_with_solInfo.mat',"RM")