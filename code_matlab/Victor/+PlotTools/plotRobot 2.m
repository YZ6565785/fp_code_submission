% PLOT robot and RM
function robot = plotRobot(robot, varargin)
%   required:
%       robot: the rigid body tree robot model
%   opts: 
%       q_config=[0xn]: joint values
%       robot_pose=[0x4]: robot poses
%   parameters:
%       collisions: bool
%       visuals: bool

    default_q_config = [];
    default_robot_pose = zeros(1,4);
    default_collisions = 'off';
    default_visuals = 'on';
    
    p = inputParser;
    
    addRequired(p, 'robot');
    addOptional(p, 'q_config', default_q_config);
    addOptional(p, 'robot_pose', default_robot_pose);
    addParameter(p, 'collisions', default_collisions, @isstring);
    addParameter(p, 'visuals', default_visuals, @isstring);
    
    parse(p, robot, varargin{:});
    args = p.Results;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % arguments
    robot = args.robot;
    q_config = args.q_config;
    robot_pose = args.robot_pose;
    collisions = args.collisions;
    visuals = args.visuals;
    
    
    if isempty(q_config)
        show(robot, 'Collisions',collisions,'Visuals',visuals, 'Position', robot_pose);
    else
        show(robot, q_config, 'Collisions',collisions,'Visuals',visuals, 'Position', robot_pose);
    end
end