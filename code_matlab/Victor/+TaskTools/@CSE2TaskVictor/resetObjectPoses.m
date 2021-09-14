function resetObjectPoses(self)
%RESETOBJECTPOSES Summary of this function goes here
%   Detailed explanation goes here
    for i=1:size(self.objects,2)
       self.objects{1,i}.Pose = self.objects_Pose{1,i}.Pose; 
    end
end

