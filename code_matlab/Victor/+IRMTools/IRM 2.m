classdef IRM < handle
    properties
        map = [] % a list of IRM layers/maps, at each height=z.
        num_poses % number of sampling poses per sphere/voxel.
        res % resolution of the Map, interval of discreted positions.
        robot_name
        dim % [th*bl*pos] dimension description of each layer/map.

    end
    
    properties (Access = private)
        sampled_poses % a list of sampled poses on sphere, depending on num_poses.
    end
%     
    methods
        function self = IRM(n, r)
            self.map = [];
%             self.num_poses = n;
            self.res = r;
            self.robot_name = "";
            
            self.setNumPoses(n);
            
            
        end
        
        % setter methods
        function self = setNumPoses(self, n)
           self.num_poses = n; 
           self.sampled_poses = InverseReachability.zacharias(self.res, n);
        end
        
        % getter methods
        function sampled_poses = getSampledPoses(self)
           sampled_poses = self.sampled_poses;
        end
        
        % a list of heights of the irm layers
        function Z = getZ(self)
            fun = @(id) str2double(self.map(id).z);
            Z = arrayfun(fun, 1:size(self.map,1))';
        end
        
        function map = findMap(self, z)
%            map.z = num2str(z);
%            map.bl = [];
%            map.bl_sum = [];
%            for i=1:size(self.map)
%                if abs(str2double(self.map(i).z) - z) <0.05
%                    map = self.map(i);
%                    break;
%                end
%            end
            map.z = num2str(z);
            map.bl = [];
           
            ind = find(abs(self.getZ()-z)<(self.res/2));
            if isempty(ind)
                map.z = num2str(z);
                map.bl = [];
            else
                map = self.map(ind); 
            end

        end
        
        
        
    end
end