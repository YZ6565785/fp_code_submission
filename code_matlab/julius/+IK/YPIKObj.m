classdef YPIKObj < IK.IKObj

    methods

        function self = YPIKObj()
            self = self@IK.IKObj();
            self.group = "youwasp_arm";
%             self.group = "youpanda_arm";
            self.ee_link = "extruder_ee";
            self.joint_names = {'vomni_joint_x', 'vomni_joint_y', 'vomni_joint_z', 'panda_joint1', 'panda_joint2', 'panda_joint4', 'panda_joint6'};
%             self.joint_names = {'vomni_joint_x', 'vomni_joint_y', 'vomni_joint_z', 'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'};
            self.base_to_arm = [0.167 0 0];
            self.setup_default_ik_msg()% setup the reusable obj msg
            self.setup_default_cart_msg()% setup the reusable obj msg
            self.reach = 1.5;
%             self.load_robot();
        end

        function seed = get_default_seed(self)
            seed = deg2rad([-10; 45;  -102; 145]);
%             seed = deg2rad([-10; 45; 10; -102; -10; 145; -166]);
            
        end

%         function load_robot(self)
%             self.robot_obj = loadrobot("frankaEmikaPanda");
%         end

        function manip = calc_manip(self,jnt_tr)
            robotRBT = loadrobot("frankaEmikaPanda")
            
            config = homeConfiguration(robotRBT);
            [n, m] = size(jnt_tr);
            manip = zeros(n, 1);

            for ii = 1:n

                for jj = 1:m
                    config(jj).JointPosition = jnt_tr(ii, jj);
                    J = geometricJacobian(robotRBT, config, "panda_link8");
                    J=J(1:3,:);
%                     J=J(1:5,:);
%                     manip(ii) = sqrt(det(J * (J')));
                    manip(ii) = sqrt(det(J * (J')));
%                     manip(ii) = sqrt(det(J * (J')));
                    % TODO remember that I am not loading from URDF here so the link is a bit  lower etc.  but thats just fixes transform.
                end

            end

        end

    end

end
