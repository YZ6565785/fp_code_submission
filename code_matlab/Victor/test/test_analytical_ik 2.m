

ROBOT_MODEL = 'xarm6_box_base2_ee';
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
robot = importrobot(robot_path);
robot.DataFormat = 'column'
show(robot);

homeConfiguration(robot)


ik = analyticalInverseKinematics(robot)
showdetails(ik)
ik.KinematicGroup

%%
robot = loadrobot("universalUR5", 'DataFormat','row')
show(robot);
%%
robot = loadrobot("universalUR3", 'DataFormat','row')
show(robot);
showdetails(robot);
ik = analyticalInverseKinematics(robot,'KinematicGroup', ...
    struct("BaseName","base_link","EndEffectorBodyName","forearm_link"))
showdetails(ik)
ik.KinematicGroup
%%
ik.KinematicGroup

generateIKFunction(ik,'robotIK');




