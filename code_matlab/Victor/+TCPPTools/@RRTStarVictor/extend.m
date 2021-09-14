function [q_new,reached] = extend(self, q_best, q_rand, varargin)
%EXTEND Summary of this function goes here
%   Detailed explanation goes here
%         if self.config.isValid(q_rand) % no need to check again, cause
%         some ik solutions may collide with obstacles, iterative ik solution
%         cant guarantee the same.

    rr = self.e_reach;

    if ~isempty(varargin)
        rr = varargin{1};
    end
            
    q_new = TaskTools.Q2Pi(q_best.q);

    assert(q_best.q(3)>=0 & q_best.q(3)<=2*pi);

    ddq = self.e_inc * (q_rand - q_best)/norm(q_rand - q_best); % note the norm would include time.

    while q_rand.dist(q_new) > self.e_inc 
        % q_new is more than one increment away

        q_new.q = q_new.q + ddq;
        q_new.q(3) = wrapTo2Pi(q_new.q(3)); % convert to [0,2*pi]

        if ~self.config.isValid2d(q_new) || q_new.dist(q_best) > rr
            q_new.q = q_new.q - ddq;
            q_new.q(3) = wrapTo2Pi(q_new.q(3)); % convert to [0,2*pi]
            if q_new.q(4) <= q_best.q(4)
               reached = -1; 
            else
               reached = 0; 
            end
            return;
        end

    end

    reached = 1;
end