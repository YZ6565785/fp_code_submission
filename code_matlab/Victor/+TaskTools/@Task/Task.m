classdef Task < Tasks.Path % inherited from Julius's Task pkg
    
    properties
        T
        points
        density = 0.001 % sampling interval - distance between two task position
    end
    
    methods

        function self = Task(points)        
            % constructor not in use, only functions
            self = self@Tasks.Path(points); 
            self.points = points;
            self.path = points;
        end
        
        function tform = pointsToTForm(self, varargin)
            % path nx3; tform 4x4xn. Turns a path into a trajectory. Assumes no rotation about local X axis. X axis pointed along the path.
            path_ = self.points;
            n = length(path_);
            ax = diff(path_, 1);
            ax = [ax; ax(end, :)];
            tform = zeros(4, 4, n);
            
            for ii = 1:n
                %Follow the curve and compute rotations in the appropriate frame
                ax0 = ax(ii, :);
                ez = atan2(ax0(2), ax0(1));
                rotmz = eul2rotm([ez, 0, 0], "ZYX");
                
                if (~isempty(varargin) && varargin{1} == "zonly")
                    ey = 0;
                else
                    ax1 = (rotmz) \ ax0';
                    ey = -atan2(ax1(3), ax1(1));
                end
                
                rotmy = eul2rotm([0, ey, 0], "ZYX");
                tform(:, :, ii) = rotm2tform(rotmz * rotmy);
                tform(1:3, 4, ii) = path_(ii, :)';
                
            end
        end
        
        function impose(self, pattern)
            n_points_in_pattern = size(pattern.path,1);
            assert(~isempty(pattern.len), "Pattern need a property len");

            cl = self.cumlen(); % cumulative length
            num = floor(cl(end)/pattern.len);
            num_mod = mod(cl(end),pattern.len);

%             density = 0.01;
            assert(~isempty(self.density), "Class need a property density");
            n_samples = 1/self.density;
            path = zeros(round(num*n_samples + (num_mod/self.density)), 3);

            x = linspace(0,1,n_points_in_pattern)';

            for i=1:ceil(cl(end)/pattern.len)
                i0 = (i-1)*n_samples+1;
                if i<=num
                    iend = i*n_samples; 
                    xq = linspace(0,1,n_samples)';
                else
                    iend = i0+num_mod/self.density-1;  
                    xq = linspace(0,num_mod,num_mod/self.density)';
                end

                interp_fun = @(a) interp1(x, pattern.path(:,a), xq,'pchip'); % pchip: cubic approx
                xs = interp_fun(1);
                ys = interp_fun(2);
                zs = interp_fun(3);

                path(i0:iend,:) = [xs+pattern.len*(i-1) ys zs];
            end 
            self.path = path;
        end
        
        % x-axis direction not changing
        function tform = toTForm2(self, varargin)

            points_T = self.pointsToTForm();
            
            path_ = self.path;
            n = size(path_,1);
            
            tform = zeros(4, 4, n);
            for i = 1:n
                
                [~,I] = pdist2(self.points, path_(i,1:3), 'euclidean', 'Smallest', 1);
                
                tform(:,:,i) = points_T(:,:,I);
                tform(1:3, 4, i) = path_(i, :)';

            end

        end
        % x-axis direction changes!!
        function tform = toTForm3(self, varargin)

            points_T = self.pointsToTForm();
            
            path_ = self.path;
            n = size(path_,1);
            
            ax = diff(path_, 1);
            ax = [ax; ax(end, :)];
            
            tform = zeros(4, 4, n);
            for i = 1:n
                
                [~,I] = pdist2(self.points, path_(i,1:3), 'euclidean', 'Smallest', 1);
                
                % apply rotation around z-axis, to change direction of
                % x-axis
                ax0 = ax(i, :);
                ez = atan2(ax0(2), ax0(1));
                rotmz = eul2rotm([ez, 0, 0], "ZYX");
                
                
                tform(:,:,i) = points_T(:,:,I);
                tform(1:3,1:3,i) = tform(1:3,1:3,i)*rotmz;
                tform(1:3, 4, i) = path_(i, :)';

            end

        end
        
        function superimpose(self, pattern)
            
            % start to impose patterns
            cl_path = self.cumlen();
            self.T = self.toTForm(self);
            vec = TForm.tform2vec(self.T); % path poses in vec form

            f_quat_path = @(l) quatnormalize(interp1(cl_path, vec(:, 4:end), l));
            f_quat_path(cl_path(end));

            f_xyz_path = @(l) interp1(cl_path, vec(:, 1:3), l);
            f_tform = @(l) TForm.vec2tform([f_xyz_path(l) f_quat_path(l)]); % bad interpolation but will work for now

            %
            q = pattern;
            %
            cl_ptrn = q.cumlen();
            f_xyz_ptrn = q.asfun();


            f_x_norm_ptrn = @(t) interp1(cl_ptrn,q.path(:,1)./max(q.path(:,1)),t);

            path_ptrn_length = cl_ptrn(end) * round(cl_path(end)/max(q.path(:,1)));

            
            ll = 0:self.density:path_ptrn_length;
            path_new = zeros(length(ll),4);
            % T_new = zeros(4,4,length(ll));

            for i=1:length(ll)
                path_new_l = ll(i);

                ptrn_l = mod(path_new_l,cl_ptrn(end)); % for each new path point, what len of the ptrn fits this point

                ptrn_x_norm = f_x_norm_ptrn(ptrn_l); % get the x_progress from the ptrn lenght

                ptrn_l_from_x_norm = ptrn_x_norm * cl_ptrn(end); % length based on progress


                path_l_from_x_norm = path_new_l - ptrn_l + ptrn_l_from_x_norm; % path length from ptrn x norm
                path_l_from_x_norm = path_l_from_x_norm * max(q.path(:,1)) / cl_ptrn(end); % normlize based on pattern

            %     data(i,1) = path_l_from_x_norm;

                if path_l_from_x_norm > cl_path
                    disp(">>>> superimpose done!");
                    path_new = path_new(1:i-1,:);
            %         T_new = T_new(:,:,1:i-1);
                    break
                end

                T = f_tform(path_l_from_x_norm);

            %     ind = round(path_l_from_x_norm/cl_path(end)*(size(cl_path,1)-1))+1;
            %     T = self.T(:,:,ind)

                p = f_xyz_ptrn(ptrn_l)';
                p(1) = 0;

                T(1:4,4) = T * [p;1];

                path_new(i,1:3) = T(1:3,4)';
            end

            self.path = path_new(:,1:3);
            self.T = self.toTForm2();

        end

    end
    
    
        
end
