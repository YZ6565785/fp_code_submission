classdef WavePath < TaskTools.Task
    %WAVEPATH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sl
        sh
    end
    
    methods
        function self = WavePath()
            %WAVEPATH Construct an instance of this class
            %   Detailed explanation goes here
            t = linspace(-2, 1, 100)';
            points = [t zeros(size(t)) sin(linspace(0,6*pi-pi/2, 100)')*0.25+0.25];
            
            t = linspace(0,1,100)';
            points = [points; sin(t)+1, cos(t)-1, zeros(size(t))];
%             t = linspace(1, 0, 100)';
%             points = [points; t zeros(size(t)) zeros(size(t))];
            

            self = self@TaskTools.Task(points); 
            self.density = 0.01;
            
%             self.impose(S);
%             self.resample(self.density);
%             self.path = flip(self.path,1);
            
%             self.path(:,3) = self.path(:,3)+S.y_rad;
            self.resample(self.density);
            self.points = self.path;
            
            q = Tasks.UPath(self.density*2);
            q.scale([0.2 0.05 1]);
            self.superimpose(q);
            
            
            

            self.T = self.toTForm2();
            
            
        end
    end
end

