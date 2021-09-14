function  viewTaskPoses(task)
%VIEWTASKPOSES Summary of this function goes here
%   Detailed explanation goes here
    
    
    PlotTools.plotPoses( TForm.tform2vec(task.T), 0.2);
end

