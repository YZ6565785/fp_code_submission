function H = plotIRMLayer(map, alpha, sphere_size, show_colorbar, show_msg)
    if ~exist('show_colorbar', 'var')
        show_colorbar = false;
    end
    if ~exist('alpha', 'var')
        alpha = 1;
    end
    if ~exist('sphere_size', 'var')
        sphere_size = 110;
    end
    
    if ~exist('show_msg', 'var')
        show_msg = true;
    end
    
    total_num_poses = sum(map.bl(:, 1, :),'all');
    if show_msg
        disp("found " + total_num_poses + " poses in this IRM at z="+map.z);
    end
    if total_num_poses == 0 && show_msg
%         disp("So, not showing any.");
        title("No IRM available");
    else
        
        H = sum(map.bl(:,1,:), 1);
        base_X = map.bl(1,2,:);
        base_Y = map.bl(1,3,:);
        base_Z = map.bl(1,4,:);
        
        indices = H > 0;
        H = H(indices);
        base_X = base_X(indices);
        base_Y = base_Y(indices);
        base_Z = base_Z(indices);

        base_color = PlotTools.generateRGB(H, max(H));
        s = scatter3(base_X, base_Y, base_Z, sphere_size, base_color, 'filled');
        s.LineWidth = 0.6;
        s.MarkerEdgeColor = 'b';
        s.MarkerEdgeAlpha = alpha;
        s.MarkerFaceAlpha = alpha;
        if show_colorbar
            colorbar
        end
        axis equal
        title("IRM at z="+map.z);
        
    end
end