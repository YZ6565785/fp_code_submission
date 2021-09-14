robot_path = "../src/xarm_ros/xarm_description/robots/xarm6_box_base2.urdf";
robot = importrobot(robot_path)
PlotTools.plotRobot(robot)
%%
robot = importrobot(robot_path)
robot.DataFormat = 'column'
q_config = [0 0.0 0 0 0 0 ]';
checkCollision(robot, q_config)
PlotTools.plotRobot(robot, q_config)

%% collision check test with world objs
robot_path = "../src/xarm_ros/xarm_description/robots/xarm6_box_base2.urdf";
test_joint_values = [];
test_joint_values = [test_joint_values; 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ];
test_joint_values = [test_joint_values; 0.0, 1.03, 0.0, 0.0, 0.0, 0.0 ];
test_joint_values = [test_joint_values; 0.0, 0.0, 0.19, 0.0, 0.0, 0.0 ];
test_joint_values = [test_joint_values; 0.0, 0.0, 0.0, 0.0, 1.33, 0.0 ];
test_joint_values = [test_joint_values; 0.0, 0.0, 0.0, 0.0, 1.31, 0.0 ];
test_joint_values = [test_joint_values; 0.0, 0.0, 0.0, 0.0, 1.29, 0.0 ];
test_joint_values = [test_joint_values; 0.0, -1.05, -0.93, 0.0, 0.0, 0.0 ];
test_joint_values = [test_joint_values; 0.0, 1.3, -1.3, 0.0, 0.0, 0.0 ];
test_ans = [false true true true true false true, true];

disp("Test Collision Checking")

box = collisionBox(0.2,0.2,0.2);
box_T = eye(4);
box_T(1:3,4) = [0.5, 0, 0.2];
box.Pose = box_T;

for i=1:size(test_joint_values,1)
    q_config = test_joint_values(i, :)';
    
    in_collision = checkCollision(robot, q_config, {box});
    test_msg = "Test ["+i+"]: ";
    if any(in_collision) == test_ans(i)
        result_msg = test_msg+'PASSED';
    else
        result_msg = test_msg+'FAILED';
    end
    disp(result_msg);
    PlotTools.plotRobot(robot, q_config);
    title(result_msg)
    hold on
    show(box)
    hold off
    axis([-1 1 -1 1 -1 1])
    drawnow
    pause(.1);
    
end

%% check collision with world object
n = 100;
joint_vals = linspace(0, 1.3, n);
test_joint_values = zeros(n, 6);
test_joint_values(:, 2) = joint_vals;
test_joint_values(:, 3) = -joint_vals;


box = collisionBox(0.2,0.2,0.2);
box_T = eye(4);
box_T(1:3,4) = [0.5, 0, 0.2]
box.Pose = box_T;

figure(2)
axis([-1 1 -1 1 -1 1])
for i=1:size(test_joint_values,1)
    q_config = test_joint_values(i, :)';
    
    in_collision = checkCollision(robot, q_config, {box});
    
    if ~any(in_collision)
        PlotTools.plotRobot(robot, q_config);

        hold on
        show(box)
        hold off
        
%         pause(0.01);
%         view(0,0);
        drawnow
    end
end
%% check robot collision with multiple world objects
robot_path = "../src/xarm_ros/xarm_description/robots/xarm6_box_base2.urdf";
robot = importrobot(robot_path)
robot.DataFormat = 'column'
q_config = homeConfiguration(robot);



box = collisionBox(0.2,0.2,0.2);
box_T = eye(4);
box_T(1:3,4) = [0.5, 0, 0.2]
box.Pose = box_T;
box2 = collisionBox(0.2,0.2,0.2);
box_T(1:3,4) = [0.2, 0, 0.2]
box2.Pose = box_T;
figure(2); clf
show(robot, q_config)
hold on
show(box)
show(box2)

hold off


world_objects{1} = box;
world_objects{2} = box2;
world_objects
in_collision = checkCollision(robot, q_config, world_objects)
%%
box = collisionBox(0.1,0.1,0.1);
box_T = eye(4);
box_T(1:2,4) = [0.5;0.5]

box.Pose = box_T;
figure(2)

show(box)

%%
box_T = eye(4);
objects = {};


% box_size = [0.1 0.1 0.1];
positions = [0 0; 0.3 0.3; 0.5 0.5];
n = length(positions)
for i=1:n
    box = collisionBox(0.1,0.1,0.1);
    box_T(1:2,4) = positions(i)'
    box.Pose = box_T;
    objects{i} = box;
end

figure(2); clf

hold on
for i=1:length(objects)
    show(objects{i});
end
hold off
axis equal
 