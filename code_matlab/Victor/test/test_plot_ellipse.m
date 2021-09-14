
x_rad = 2;
y_rad = 1;
length = 1;

n_points = 100;
t = linspace(0,1,n_points+1)';
t = t(1:end-1);
assert(size(t,1)==n_points, "Make sure t is a "+n_points+"-by-1 matrix");

x = sin(pi*2*t)*x_scale
y = cos(pi*2*t)*y_scale
z1 = zeros(size(t))+length/2


z2 = zeros(size(t))-length/2

figure(1);clf;hold on
scatter3(x,y,z1)
scatter3(x,y,z2)

V = [x y z1; x y z2]
elliptic_cylinder = collisionMesh(V)
show(elliptic_cylinder)

axis equal

%% check elliptic cylinder class collision correctness
x_rad = 0.2;
y_rad = 0.1;
length = 0.4;
elliptic_cylinder = CollisionEllipticCylinder(x_rad, y_rad, length);
elliptic_cylinder.Pose = elliptic_cylinder.Pose * eul2tform([0 0 pi/2])

ec1 = CollisionEllipticCylinder(x_rad, y_rad, length);
ec1.Pose = ec1.Pose * eul2tform([0 0 pi/2])
ec2 = CollisionEllipticCylinder(x_rad, y_rad, length);
ec2.Pose(3,4) = 0.301
checkCollision(ec1, ec2)

figure(2);clf;hold on
show(ec1)
show(ec2)
axis equal

disp('TEST PASSED!')