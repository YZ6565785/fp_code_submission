classdef QTreeOrdered < handle
    %QTREEORDERED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        array
        arrval
        idx
        sz
        grow_ratio = 1.5;
    end
    
    methods
        function self = QTreeOrdered(q_arr)
            %QTREEORDERED Construct an instance of this class
            %   Detailed explanation goes here
            
            q_mat = vertcat(q_arr.q);
            progress = q_mat(:, 4);

            [progress, I] = sort(progress);
            self.arrval = progress;
            self.array = q_arr(I);
            self.sz = length(self.array);
            self.idx = self.sz;
        end
        
    end
end

