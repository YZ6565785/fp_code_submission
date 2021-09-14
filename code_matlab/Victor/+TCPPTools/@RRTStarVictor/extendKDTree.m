function [q_new,reached] = extendKDTree(self, q_best, q_rand, varargin)
%EXTEND Summary of this function goes here
%   Detailed explanation goes here
%         if self.config.isValid(q_rand) % no need to check again, cause
%         some ik solutions may collide with obstacles, iterative ik solution
%         cant guarantee the same.

    rr = self.e_reach;

    if ~isempty(varargin)
        rr = varargin{1};
    end
            
    q_new = TaskTools.Q2PiKDTree(q_best.q);
    q_new.pose_sol = q_best.pose_sol;

    assert(q_best.q(3)>=0 & q_best.q(3)<=2*pi);


    ddq = self.e_inc * (q_rand - q_best)/norm(q_rand - q_best); % note the norm would include time.
    
%     if ddq(4) <0
%         
%         disp('test: neg task progress movement')
%     end
    ddq(4) = abs(ddq(4));
    reached = 1;
    while q_rand.dist(q_new) > self.e_inc 
        % q_new is more than one increment away

        q_new_next = TaskTools.Q2PiKDTree(q_new.q + ddq);

        if ~self.config.isValid2d(q_new_next) || q_new_next.dist(q_best) > rr
%             disp("invalid in 2D")
%             if q_new_next.q(4) <= q_best.q(4) 
%                 reached = -1; 
%             else
%                 reached = 0;
%                 
%             end
            reached = 0;
            break;
        end
        q_new = q_new_next;
    end
%     if ~self.config.isValid3d(q_new, q_best.pose_sol)
%         reached = -1;
%         return
%     end

    while ~self.config.isValid3d(q_new, q_best.pose_sol) 
        if q_new.q(4)<=q_best.q(4)
            break
        end
%         disp("invalid Final")
        q_new.q = q_new.q-ddq;
    end
    
    
end