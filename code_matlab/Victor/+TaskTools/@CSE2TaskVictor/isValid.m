function valid = isValid(self, q, q0)
%ISVALID Summary of self function goes here
%   Detailed explanation goes here
%   q: a simple sample

   
    if ~self.isValid2d(q)
        valid = false;
        return;
    end
    
    if ~exist('q0','var')
        q0 = homeConfiguration(self.robot);
    end
    valid = self.isValid3d(q,q0);
end