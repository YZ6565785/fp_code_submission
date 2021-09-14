%% generate mask from orientation
function [mask] = generateMask(samples, axis, positive, th)
    if ~exist('th','var')
       th = 60; 
    end

    TT=TForm.vec2tform(samples);
    
    % towards x axis -> axis = 1
    % towards y axis -> axis = 2
    % towards z axis -> axis = 3
    
    if positive
        mask = acosd(min(1,squeeze(abs(TT(axis,3,:)))))<th &...
            squeeze(TT(axis,3,:)>0);
    else
        mask = acosd(min(1,squeeze(abs(TT(axis,3,:)))))<th &...
            squeeze(TT(axis,3,:)<0);
    end
end