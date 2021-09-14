function irm = computeIRM(bool_indices, res_IRM)
    disp("Computing IRM");
    base_z = 0;
    r = 1; % length that arm reaches
%     deg = 9;
%     th_interval = deg/180*pi;
    num_poses = size(bool_indices,2)-4;
    num_pos = (2*r/res_IRM+1)^2;
    num_orientations = 180/9*2;
    num_bl = size(bool_indices,2) + 1;
   
    
    th_list = linspace(0, 2*pi, num_orientations+1);
    z_itr = res_IRM;
    progress = 1;
    total = num_orientations*(2*r/res_IRM+1)^2*(2*r/z_itr+1);
    disp("Total poses: "+ total);
    check = round(total/10);


    irm = IRMTools.IRM(num_poses, res_IRM);
        
    irm.dim.th = num_orientations;
    irm.dim.bl = num_bl;
    irm.dim.pos = num_pos;
    for z=-0.5:z_itr:1.5 % z coord for task
        bl_IRM = zeros(num_orientations, num_bl, num_pos);
        j = 1;
        for h=-r:res_IRM:r
            for w = -r:res_IRM:r
                x = w; % x coord for base placement
                y = h; % y coord for base placement
                
                for i=1:num_orientations
                    th = th_list(i);

                    R = Helpers.rot2d(th-pi);
                    coord = R\[x; y];
                    ind = find(abs(bool_indices(:,2) - coord(1))<=(res_IRM/2) &...
                        abs(bool_indices(:,3) - coord(2))<=(res_IRM/2) &...
                        abs(bool_indices(:,4) - z)<=(z_itr/2), 1);

                    if ~isempty(ind)
                        bl = bool_indices(ind,5:end);
                        
                        bl_IRM(i, :, j) = [sum(bl) x y base_z th bl];
                        
                    else
        %                 disp("not found in RM");
                        bl_IRM(i, :, j) = [0 x y base_z th zeros(1, num_poses)];
                    end
                    progress = progress + 1;
                    if mod(progress, check)== 0
                        disp("Progress: " + round(progress/total*100) + "%");
                    end
                end
                
                j = j + 1;
            end

        end
        % store IRM
        s = struct;
        s.z = num2str(z);
        s.bl = bl_IRM;
%         s.bl_sum = bl_IRM_sum;
        irm.map = [irm.map; s];

    end
    % size(bl_IRM)       
    % size(bl_IRM_sum)
    disp("Done!");
    disp("IRM Resolution: " + res_IRM);

    
end