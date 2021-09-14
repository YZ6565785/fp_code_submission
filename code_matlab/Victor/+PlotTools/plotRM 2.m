function plotRM(robot_path, sphere_X, sphere_Y, sphere_Z, H, robot_pose)
    % CONVERT SCALAR TO COLOR (rgb) - scaling from red to blue (0-100)
    if~exist('robot_pose','var')
       robot_pose = zeros(1,4); 
    end
    
    sphere_color = PlotTools.generateRGB(H, max(H));

    % PLOT robot and RM
    robot = importrobot(robot_path);
    PlotTools.plotRobot(robot, 'robot_pose',robot_pose);
    hold on
    s = scatter3(sphere_X', sphere_Y', sphere_Z', 110, sphere_color, 'filled');
    s.LineWidth = 0.25;
    s.MarkerEdgeColor = 'b';
    title('Reachability Map and Robot');
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
    axis equal
    
end