function[sphere_color] = generateRGB(H, highest)
% CONVERT SCALAR TO COLOR (rgb) - scaling from red to blue (0-100)
% input
%       H: A list of scalars 
    if ~exist('highest','var')
        highest = max(H);
    end

%     old implementation
%     scalars = -H;
%     minimum = min(scalars);
%     maximum = max(scalars);
% 
%     ratios = 2 * (scalars-minimum) / (maximum - minimum); % (1,n)
% 
%     B = max([(1-ratios); zeros(1, size(scalars,2))], [], 1);
%     R = max([(ratios-1); zeros(1, size(scalars,2))], [], 1);
%     G = (1-B-R);
%     sphere_color = [R;G;B]';
    
%     new implementation
    if highest==0
        ind= 1;
    else
        ind = round(H/highest*255)+1;
    end
    map= flipud(colormap("parula"));
    sphere_color = map(ind,:);
    
end