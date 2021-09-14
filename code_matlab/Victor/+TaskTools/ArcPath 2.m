classdef ArcPath < TaskTools.Task
    %ARCPATH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function self = ArcPath(angle)
            %ARCPATH Construct an instance of this class
            %   Detailed explanation goes here
            t = linspace(0, 1, 100)';
            points = [cos(angle * t) sin(angle * t) zeros(100, 1)];         
            self = self@TaskTools.Task(points);
            self.density = 0.01;
            
            self.scale([1, .5, 1]);
            self.resample(self.density);
            self.points = self.path;
            
            
            q = Tasks.UPath(.1);
            q.scale([0.2 0.075 1]);
            self.superimpose(q);
            
            self.T = self.toTForm2();
           	
        end
    
    end
end

