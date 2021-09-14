
%% 
robot_name = "xarm6_box_base2_ee";
%set robot path
robot_path = "../xarm_fp/xarm_description/robots/"+robot_name+".urdf"

f=figure(1);
robot = importrobot(robot_path);
PlotTools.plotRobot(robot, 'robot_pose',[0 0 0 0]);
view([15 30])
axis([-1 1 -1 1 0 1])

if false
    saveas(f, "../UCL Final Project/figures/task/robot_model.png", 'png')
end