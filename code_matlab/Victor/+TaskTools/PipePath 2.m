classdef PipePath < TaskTools.Task
    %PIPEPATH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function self = PipePath(pipe_x, pipe_rad)
            %PIPEPATH Construct an instance of this class
            %   Detailed explanation goes here
%             pipe_rad = [0.1 0.1]; % 1st rad for x-axis, 2nd rad for z-axis
%             pipe_x = 0.7;
            assert(pipe_x>=0 && pipe_x<=1, "center of the pipe must be between 0 and 1.")
            assert(pipe_rad(1)>=0 && pipe_rad(1)<=1, "x rad of the pipe must be between 0 and 1.")
            assert(pipe_rad(2)>=0 && pipe_rad(2)<=1, "y rad of the pipe must be between 0 and 1.")
            
            angle = pi;
            
            
            
            x = linspace(pipe_x+pipe_rad(1)+1, pipe_x+pipe_rad(1), 10)';
            t = linspace(0,1,10)';
            points = [sin(t)+pipe_x+pipe_rad(1) cos(t)-1  zeros(size(x)) ];
            
            t = linspace(0, 1, 50)';
            points = [flip(points(1:end-1,:),1); cos(angle * t)*pipe_rad(1)+pipe_x zeros(size(t)) sin(angle * t)*pipe_rad(2)];
            
            x = linspace(pipe_x-pipe_rad(1), pipe_x-pipe_rad(1)-1, 10)';
            points = [points; x zeros(size(x)) zeros(size(x))];
            
%             figure(1);clf;hold on;axis equal
%             scatter3(points(:,1),points(:,2),points(:,3));
%             points

            self = self@TaskTools.Task(points);
            self.path = points;  
            self.density = 0.005;
            
            self.resample(self.density);
            self.path = flip(self.path,1);
            self.points = self.path;
            
            
            q = Tasks.UPath(self.density*2);
            q.scale([0.2 0.05 1]);
            self.superimpose(q);
            self.resample(self.density);
            
            self.T = self.toTForm2();
        end
       
        
        function T = getT(self)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            path_ = self.path;
            n = size(path_,1);
            ax = diff(path_,1);
            ax = [ax; ax(end,:)];
            T = zeros(4,4,n);
            
            Ty = self.pointsToTForm();
            n_points = size(self.points,1);
            
            for i=1:n
               ax0 = ax(i,:);
               ez = atan2(ax0(2), ax0(1));
               rotmZ = eul2rotm([ez,0,0], 'ZYX');
               
               ind = ceil(i/n*n_points);
               rotmY = tform2rotm(Ty(:,:,ind));
               
               T(:,:,i) = rotm2tform(rotmZ*rotmY);
               T(1:3,4,i) = path_(i,:)';
            end
            
        end
        
        
    end
end

