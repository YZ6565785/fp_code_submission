

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
%%
robot = loadrobot('universalUR5');
robot.DataFormat = 'column';
show(robot, 'Collisions','on','Visuals','on');

showdetails(robot);
ee = "ee_link";

%%
ik = inverseKinematics("RigidBodyTree", robot);
weights = ones(1,6);


q0 = homeConfiguration(robot);
q0

pose1 = TForm.DOWN;
    pose1(1:3, 4) = [0, 0, 0.17];
[poseSol, poseInfo] = ik(ee, pose1, weights, q0);
poseSol
poseInfo

sol = poseSol;
show(robot, sol, 'Collisions','on','Visuals','off');
isColliding = checkCollision(robot,sol);
printCollision(isColliding);

%% for iiwa14 robot
robot = importrobot('iiwa14.urdf');
robot.DataFormat = 'column';
show(robot, 'Collisions','on','Visuals','off');

showdetails(robot);
ee = "iiwa_link_ee_kuka";

%%
robot = importrobot('sawyer.urdf');
robot.DataFormat = 'column';
show(robot, 'Collisions','on','Visuals','off');

showdetails(robot);
ee = "panda_hand";

%%
ik = inverseKinematics("RigidBodyTree", robot);
weights = ones(1,6);


q0 = homeConfiguration(robot);
q0

pose1 = TForm.DOWN;
pose1(1:3, 4) = [0., 0., 0.15];
[poseSol, poseInfo] = ik(ee, pose1, weights, q0);
poseSol
poseInfo

sol = poseSol;
show(robot, sol, 'Collisions','on','Visuals','off');
isColliding = checkCollision(robot,sol)
% printCollision(isColliding);

