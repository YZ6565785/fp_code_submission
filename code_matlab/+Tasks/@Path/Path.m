classdef (Abstract) Path < handle

    properties
        path% nx3 path
        traj%4x4xn trajectory
    end

    methods

        function self = Path(seed_path)
            self.path = self.normalise_path(seed_path);
            % self.traj = self.normalise_path(seed_path);

        end
        

        function scale(self, scale_vec)
            % scale  Scale the path along xyz axes
            self.path = self.normalise_path(self.path);
            self.path(:, 1) = self.path(:, 1) * scale_vec(1);
            self.path(:, 2) = self.path(:, 2) * scale_vec(2);
            self.path(:, 3) = self.path(:, 3) * scale_vec(3);
        end

        function resample(self, density)
            cumlength = self.cumlen();
            [~, IA, ~] = uniquetol(cumlength);
            cumlength = cumlength(IA);
            self.path = self.path(IA, :);
            self.path = interp1(cumlength, self.path, linspace(0, cumlength(end), round(cumlength(end) / density)), 'pchip');
        end

        function superimpose(self, pattern)
            
            % start to impose patterns
            cl_path = self.cumlen();
            T = self.toTForm(self);
            vec = TForm.tform2vec(T); % path poses in vec form

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

            density=0.01;
            ll = 0:density:path_ptrn_length;
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
%             self.T = self.toTForm2();

        end

        function h = asfun(self)
            l = self.cumlen();
            % l = l ./ l(end);
            h = @(t) interp1(l, self.path, t, 'pchip');
        end

        function t = gett(self, speed)
            l = self.cumlen();
            t = l ./ speed;
        end

        function cumlength = cumlen(self, varargin)
            path_ = self.path;

            if ~isempty(varargin)
                path_ = varargin{1};
                path_ = path_.path;
            end

            % Compute cumulatative length of the path
            cumlength = [0; cumsum(sqrt(sum(diff(path_).^2, 2)))];
        end

        function valid = is_repeating(self)
            valid = all(abs(self.path(1, 2:3) - self.path(end, 2:3)) < 1e-5);
        end

        function h = plot(self, varargin)
            iend = size(self.path(:, 1), 1);

            if ~isempty(varargin)
                iend = varargin{1}
            end

            % scale  Scale the path along xyz axes
            x = self.path(1:iend, 1);
            y = self.path(1:iend, 2);
            z = self.path(1:iend, 3);

            c = colormap("winter");
            l = (1:length(x)) ./ length(x);
            c = interp1(linspace(0, 1, length(c)), c, l);
            h = scatter3(x, y, z, 5, c, 'filled');
        end

        function smooth(self, s_ratio)
            s = size(self.path);
            x = smooth(self.path(:, 1), round(s(1) * s_ratio));
            y = smooth(self.path(:, 2), round(s(1) * s_ratio));
            z = smooth(self.path(:, 3), round(s(1) * s_ratio));
            self.path = [x y z];
        end

    end

    methods (Static)

        function tform = toTForm(path_, varargin)
            % path nx3; tform 4x4xn. Turns a path into a trajectory. Assumes no rotation about local X axis. X axis pointed along the path.
            path_ = path_.path;
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

        function path_ = fromTForm(tform)
            % path nx3; tform 4x4xn. Turns a path into a trajectory. Assumes no rotation about local X axis. X axis pointed along the path.
            path_ = squeeze(tform(1:3, 4, :))';
        end

        function h = plotTForm(tform, frame_density)
            x = squeeze(tform(1, 4, :));
            y = squeeze(tform(2, 4, :));
            z = squeeze(tform(3, 4, :));

            ax = squeeze(tform(1:3, 1, :));
            ay = squeeze(tform(1:3, 2, :));
            az = squeeze(tform(1:3, 3, :));
            cumlen = [0; cumsum(sqrt(sum(diff([x y z]).^2, 2)))];
            n = find([0; diff(round(cumlen ./ frame_density))]); % find entries that go over density threshold

            c = colormap("winter");
            l = (1:length(x)) ./ length(x);
            c = interp1(linspace(0, 1, length(c)), c, l);

            h = scatter3(x, y, z, 5, c, 'filled');
            hold on
            quiver3(x(n), y(n), z(n), ax(1, n)', ax(2, n)', ax(3, n)', 'r');
            quiver3(x(n), y(n), z(n), ay(1, n)', ay(2, n)', ay(3, n)', 'g');
            quiver3(x(n), y(n), z(n), az(1, n)', az(2, n)', az(3, n)', 'b');
            hold off
        end

        function path_ = normalise_path(path_)
            % normalise  given path so that its amplitude in any direction is max 1.
            normalise = @(x) x ./ max(abs(x));
            path_ = [normalise(path_(:, 1)), normalise(path_(:, 2)), normalise(path_(:, 3))];
            path_(isnan(path_)) = 0;
        end

    end

end
