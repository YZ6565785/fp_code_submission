%%
robot = loadrobot("frankaEmikaPanda");
ee = 'panda_hand';
%%
show(robot);

%%
ik = inverseKinematics('RigidBodyTree', robot);
%% Test ik first

homeTform = TForm.DOWN;
homeTform(1:3, 4) = [0.5, 0.5, 0];
homeTform
q0 = homeConfiguration(robot);
weights = ones(1,6);
[homeSol, homeInfo] = ik(ee, homeTform, weights, q0)


