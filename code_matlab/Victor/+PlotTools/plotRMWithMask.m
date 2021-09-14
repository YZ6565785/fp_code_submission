function plotRMWithMask(robot_path, bool_indices, samples, mask, res, view_angles, method)
    if ~exist('method','var')
        method = 1;
    end
    
    bl = bool_indices(:,5:end) & mask';
    ri = sum(bl,2);
    indices = ~(bool_indices(:,2)<0.2 & bool_indices(:,4)>0.1) &...
        ri>0;
    indices = ri>0 & (abs(bool_indices(:,2)-0)<0.05 | abs(bool_indices(:,3)-0)<0.05 | abs(bool_indices(:,4)-0)<0.05);

    H = ri(indices);
    sphere_X = bool_indices(indices,2);
    sphere_Y = bool_indices(indices,3);
    sphere_Z = bool_indices(indices,4);

    
    view_u = -50;
    view_v = 30;
    % plot the robot and RM constrained here
    if exist('view_angles','var')
        view_u = view_angles(1);
        view_v = view_angles(2);
    end
    
    
    % Top plot
    if method==1
        subplot(1,2,1)

        PlotTools.plotRM(robot_path, sphere_X, sphere_Y, sphere_Z, H);

        axis equal
        title('Global Constrainted RM with Mask');
        view([view_u view_v])
    end
    
    
    % method 2, showing every pose with arrows
    if method ==2
        % find unique H, same color poses
        unique_H = unique(H);
        sphere_color = PlotTools.generateRGB(H, max(H));
        % the most fast method to plot orientations and IRM together so far
        for i_H=1:size(unique_H,1)
            h = unique_H(i_H); 

            bool_indices_show = bool_indices(indices,:);
            inds_true = H(:)==h;

            % repeat every coord xyz 'len(mask)' times
            X = reshape(repmat(bool_indices_show(inds_true,2),1,length(mask))',[],1); 
            Y = reshape(repmat(bool_indices_show(inds_true,3),1,length(mask))',[],1);
            Z = reshape(repmat(bool_indices_show(inds_true,4),1,length(mask))',[],1);
    %         [X Y Z]

            vec = repmat(samples(:,1:3), sum(inds_true), 1) * 0.8;
            bl = reshape((bool_indices_show(inds_true,5:end)&mask')',[],1);
            inds = bl(:)==1;


            X = X(inds);
            Y = Y(inds);
            Z = Z(inds);
            U = vec(inds,1);
            V = vec(inds,2);
            W = vec(inds,3);
            quiver3(X,Y,Z,U,V,W,0,'color',sphere_color(find(H==h,1),:),'LineWidth',3);   
            hold on
        end
        axis equal
        xlabel('x axis');
        ylabel('y axis');
        zlabel('z axis');
        title('Local Constrainted RM with Mask');
        view([view_u view_v])
    end 

    % plot local sphere poses with mask
    % Bottom plot
    if method == 1
        subplot(1,2, 2)
        hold on
        [X,Y,Z] = sphere;

        r = res;
        X2 = X * r;
        Y2 = Y * r;
        Z2 = Z * r;

        PlotTools.plotPoses(samples(mask,:), 0.5);
        surf(X2,Y2,Z2)


        xlabel('x axis');
        ylabel('y axis');
        zlabel('z axis');
        axis equal
        view([view_u view_v])
        axis([-0.1 0.1 -0.1 0.1 -0.1 0.1])
        title('Local Sphere Poses with Mask');
    end 
end