classdef SPath < Tasks.Path
    %SPATH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x_rad
        y_rad
        len
    end
    
    methods
        function self = SPath(length, height)
            %SPATH Construct an instance of this class
            %   Detailed explanation goes here

            angle = pi;
            x_rad = length/4;
            y_rad = height/2;

            t = linspace(0, 1, 100)';
            points = [cos(angle * t)*x_rad+x_rad zeros(size(t)) sin(angle * t)*y_rad ];
            points = [points; cos(angle * t)*x_rad-x_rad zeros(size(t)) -sin(angle * t)*y_rad ];
            % move points from 0 to length
            points = points + [x_rad*2 0 0];
            
            self = self@Tasks.Path(points); % not in use
            self.path = flip(points,1);
            self.len = length;
            self.x_rad = x_rad;
            self.y_rad = y_rad;
            
            
            
            
        end
        

    end
end

