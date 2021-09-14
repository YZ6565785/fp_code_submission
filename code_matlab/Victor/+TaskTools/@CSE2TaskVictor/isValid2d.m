function valid = isValid2d(self, q)
%isValid2d Summary of this function goes here
%   Detailed explanation goes here
    q = q.q;
    
    valid = (q(4) >= 0 || q(4) <= 1)==1;
    if ~valid 
        return
    end
    
    q_2d = (Helpers.rot2d(q(3)-pi)*self.base_vertices' + q(1:2)')';
    
    
    % object collision 2d
    self.base_2d.Vertices = q_2d;
    for i=1:size(self.objects_2d,2)
        if overlaps(self.base_2d, self.objects_2d{1,i})
            valid = false;
%             disp('speed up from checking in 2d')
            return 
        end
    end
    
    % task collision
    s = size(self.task.path, 1);
    ind_T = min(s, max(1, ceil(s * q(4))))-1;
    ind_high = self.task.path(:,3)>0.01; % non planar tasks, cannot collide
    ind_high(1:ind_T) = 1;
    xq = self.task.path(ind_high,1);
    yq = self.task.path(ind_high,2);
    
    if any(inpolygon(xq, yq, q_2d(:,1), q_2d(:,2)))
        valid = false;
        return 
    end
    
    
end

