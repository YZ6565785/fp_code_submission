classdef Q2Pi < handle
    %Q2Pi Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        parent = {}
        children = {}
        cost = 0
        iparent
        q 
        pose_sol = []
    end
    
    methods
        function self = Q2Pi(q)
            %Q2Pi Construct an instance of this class
            %   Detailed explanation goes here
%             self.q = [q(1:2) wrapTo2Pi(q(3)) min(1,max(0,q(4:end))) ];    
            self.q = [q(1:2) wrapTo2Pi(q(3)) q(4:end)];    

        end
        
        
        function r = minus(q1, q2)
            % Note the result is num vec
            angdiff = @(t, s)atan2(sin(t - s), cos(t - s));
            q1 = vertcat(q1.q);
            q2 = vertcat(q2.q);
            r = [q1(:, 1:2) - q2(:, 1:2), angdiff(q1(:, 3), q2(:, 3)) q1(:, 4) - q2(:, 4)];
        end

    end
end

