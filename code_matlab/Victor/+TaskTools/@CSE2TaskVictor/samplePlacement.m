% t: progress of the task from range [0,1]
function q = samplePlacement(self, t, varargin)
%   required:
%       t: task progress
%   opts: 
%       n=1: number of samples requires.
%   parameters:
%       show_IRM

    default_n = 1;
    default_th = 30;
    default_cut_p = 50;
    default_show_IRM = false;
    default_show_method = 1;
    default_given_mask = [];
    
    p = inputParser;
    validProgress = @(x) x>=0 && x<=1;
    validPosNum = @(x) x>0 && isnumeric(x) && isscalar(x);
    validDegreeThreshold = @(a) a>0 && a<=360;
    validCutP = @(a) a>=0 && a<100;
    
    addRequired(p, 't', validProgress);
    addOptional(p, 'n', default_n, validPosNum);
    addParameter(p, 'th', default_th, validDegreeThreshold);
    addParameter(p, 'cut_p', default_cut_p, validCutP);
    addParameter(p, 'show_IRM', default_show_IRM, @islogical);
    addParameter(p, 'show_method', default_show_method, validPosNum);
    addParameter(p, 'given_mask', default_given_mask);
    
    parse(p,t,varargin{:});
    args = p.Results;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % arguments
    t = args.t;
    n = args.n;
    th = args.th;
    cut_p = args.cut_p;
    show_IRM = args.show_IRM;
    show_method = args.show_method;
    given_mask = args.given_mask;


    T = self.getTaskFromProgress(t);

    z = T(3,4); % height of the task
    
    irm_layer = findMap(self.irm, z);
    
    % generate mask from mask
    if ~isempty(given_mask)
        mask = given_mask;
    else
        th = pi*th/180;
        T_turn = eul2tform([0 pi 0], 'ZYZ');
        u = T(1:3,1:3) * T_turn(1:3,1:3) * [0;0;1]; % unit vector along z-axis

        S = self.irm.getSampledPoses();
        mask = false(size(S,1),1);
        for i=1:size(S,1)
           v = S(i,1:3)';
           mask(i,1) = (atan2(norm(cross(u,v)), dot(u,v)) <= th);
        end
    end
    assert(sum(mask)>0,'mask cannot be false in all elements! May have higher theta threshold.')
    
    % sampling based on cummulative pdf
%     bl213 = permute(irm_layer.bl, [2 1 3]); % change dim order
%     irm_bl = reshape(bl213, self.irm.dim.bl, [])'; % to n-by-len(bl) matrix
    irm_bl = irm_layer.trueColBl; % we choose to store the true col boolean vectors to speed up the sampling
    irm_bl(:,6:end) = irm_bl(:,6:end)&mask';
    irm_bl(:, 1) = sum(irm_bl(:,6:end), 2); 
    assert(sum(irm_bl(:,1))>0,'Error, No avalable pose with this mask.');
    
    % prune irm
    if cut_p~=0
        irm_sum_min = prctile(irm_bl(irm_bl(:,1)>0, 1), cut_p); % cutoff below 50%
        irm_bl(irm_bl(:,1)<irm_sum_min,1) = 0;
    end
    
    if show_IRM
        
        if show_method==1
            % plot irm
            dim = self.irm.dim;
            bl123 = permute(reshape(irm_bl', dim.bl, dim.th, dim.pos), [2 1 3]);

            H = sum(bl123(:,1,:), 1);

            indices = H > 0;

            H = H(indices);
            bl123 = bl123(:,:,indices);
            base_color = PlotTools.generateRGB(rescale(H));
            
            unique_H = unique(H);

            % the most fast method to plot orientations and IRM together so far
            for i_H=1:size(unique_H,1)
                h = unique_H(i_H); 

                bl = reshape(permute(bl123(:,:,H==h), [2 1 3]), self.irm.dim.bl, [])';
                inds_true = bl(:,1)>0;

                X = bl(inds_true,2);
                Y = bl(inds_true,3);
                U = cos(bl(inds_true,5))*self.irm.res/2;
                V = sin(bl(inds_true,5))*self.irm.res/2;
                quiver(X,Y,U,V,0,'color',base_color(find(H==h,1),:),'LineWidth',2);   
                hold on
            end
        else
            % this method shows based on IRI of poses in every single
            % orientations
            indices = irm_bl(:,1)>0;

            H = irm_bl(indices,1);
            irm_bl = irm_bl(indices,:);
            base_color = PlotTools.generateRGB(rescale(H)); 

            unique_H = unique(H);

            for i_H=1:size(unique_H,1)
                h = unique_H(i_H); 

                inds_true = irm_bl(:,1)==h; 

                X = irm_bl(inds_true,2);
                Y = irm_bl(inds_true,3);
                U = cos(irm_bl(inds_true,5))*self.irm.res/2;
                V = sin(irm_bl(inds_true,5))*self.irm.res/2;
                quiver(X,Y,U,V,0,'color',base_color(find(H==h,1),:),'LineWidth',2);   
                hold on
            end
        end

        axis equal
        axis([-1 1 -1 1 -0.1 0.1])
        view([0 90])
        title("Pruned by "+cut_p+"%, th="+args.th+", z="+irm_layer.z+"   max="+max(H)+", min="+min(H))
    end
    
    % remove false poses, where sum(bl) = 0
    irm_bl = irm_bl(irm_bl(:,1)>0, :);
    
    cdf = cumsum(irm_bl(:, 1));
    
    % sample n placement
%     bl_list = zeros(n,3);
    q = arrayfun(@(id) TaskTools.Q2Pi([-1 -1 -1 -1]), 1:n)';
    
    for i=1:n
        r = rand() * cdf(end);
%             vec_inds = zeros(n,1);
%             for i=1:n
%                 r_each = r(i);
%                 vec_inds(i) = size(find(r_each>cdf),1)+1;
%             end
        vec_ind = size(find(r>cdf),1)+1;
        bl = irm_bl(vec_ind,:);
        % convert to q =  [x y th progress]
        bl = bl(:, [2 3 5]);
        assert(bl(:,3)>=0);
        bl(:, 1:2) = bl(:, 1:2) + T(1:2,4)'; % add offset from task
%         bl
%         bl_list(i, :) = bl;
        q(i,1) = TaskTools.Q2Pi([bl, t]);
    end
%     q = arrayfun(@(id) TaskTools.Q2Pi([bl_list(id, :), t]), 1:n)';
end