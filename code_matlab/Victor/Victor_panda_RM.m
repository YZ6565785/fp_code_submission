%%
import IK.*
import InverseReachability.*

%% LOAD ROBOT MODEL
robot = loadrobot('frankaEmikaPanda');
eeOffset = [0 0 0.1];

% init ik solver
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 200;
weights = ones(1,6); % only x and y coordinates are important
endEffector = 'panda_hand';

homePose = TForm.DOWN;
homePose(1:3,4) = [0.5, 0, 0];
q0 = homeConfiguration(robot);
[q0, solInfo]= ik(endEffector,homePose,weights,q0);
solInfo

%% %%%%%% Params %%%%%%
% params.gridrad = 1.5;
params.gridrad = 0.3;
params.gridres = 0.05;
params.sample_n = 200;
params.ikobj = @YPIKObj;
params.frame = "base_footprint";


params.trial_name = "victor_0.3rad_grid_test";
params.trial_description = "Try small grid first.";
RM.params = params;

%% create grid
idx_rad = ceil(params.gridrad ./ params.gridres);
d = (-idx_rad:idx_rad) * params.gridres;
M = length(d);
MM = M^3;
[X, Y, Z] = meshgrid(d, d, d);
RM.spans = {d, d, d};
RM.grid.X = X;
RM.grid.Y = Y;
RM.grid.Z = Z;

%% create voxels
RM.voxCenters = [X(:), Y(:), Z(:)];
RM.voxRIPoses = cell(M^3, 1);
RM.voxIKSuccess = cell(M^3, 1);
RM.voxIKSolutions = cell(M^3, 1);
RM.voxValid = false(MM, 1);
RM.voxRI = zeros(MM,1);
RM.voxRICenterSol = cell(MM,1);
RM.voxCenterSolInfo = cell(MM,1);


%% Find valid voxels. NOTE KDL with POSITION ONLY SHOULD BE RUNNING
ups = zeros(1,9);
downs = zeros(1,9);
for i=1:9
    limits = robot.Bodies{i}.Joint.PositionLimits;
    downs(i) = limits(1);
    ups(i) = limits(2);
end


POSE = repmat(TForm.DOWN, 1, 1);
check = round(MM/50);
for i=1:MM
    if mod(i,check) == 0
        disp(string(i*100/MM) + '%');
    end
    POSE(1:3, 4) = RM.voxCenters(i,:);
%     POSE
    [qSol, solInfo]= ik(endEffector, POSE, weights, q0);

    % filter out invalid solution; out of the joint limits;
    joints = [qSol.JointPosition];
    for j=1:9
        if (joints(j) > ups(j) || joints(j) < downs(j))
            disp(joints(j));
            disp(downs(j) + ", " + ups(j));
            disp("invalid solution, out of joint limits! " + string(i));
            RM.voxValid(i) = false;
        else
            RM.voxValid(i) = solInfo.ExitFlag==1;
        end
    end

    % store solutions
    RM.voxRICenterSol{i} = qSol;
    RM.voxCenterSolInfo{i} = solInfo;
%     q0 = qSol;
%     show(robot, qSol);
end
% sample_poses = TForm.tform2vec(POSES);
% ikobj = params.ikobj();
% result = ikobj.get_ik(sample_poses, "base_footprint");
% RM.voxValid = result.received_answers == 1;
% save(params.trial_name+'_with_solInfo.mat',"RM")


%%
% RM.LocalRIPoses = InverseReachability.zacharias(params.gridres / 2., params.sample_n);
% RM.LocalRIPoses = InverseReachability.double_zacharias(params.gridres / 2, params.sample_n);

% convert to tranformation matrix
poses7 = InverseReachability.zacharias(params.gridres / 2., params.sample_n);
RM.LocalRIPoses = quat2tform(poses7(:, 4:end));
RM.LocalRIPoses(1:3, 4, :) = poses7(:, 1:3)';
%% Compute Voxel Ik Calls. NOTE::: THIS MUST BE Trac ik now. with Distance.
ss=sum(RM.voxValid);
t_s=tic
check = round(MM/100);
time_taken = zeros(MM, 1);
for ii = 1:MM
    if mod(ii,check) == 0
        disp(string(ii*100/MM) + '%');
    end

    % check valid solution
    if RM.voxValid(ii)
        % sample points on a local sphere
        sample_poses = RM.LocalRIPoses;
        sample_poses(1:3, 4, :) = sample_poses(1:3, 4, :) + ...
            RM.voxCenters(ii, :)';

        % init empty array
        ExitFlags = zeros(params.sample_n,1);
        solsFound = cell(params.sample_n,1);
        for j=1:params.sample_n
%             sample_pose = sample_poses(:,:,j);
            % compute ik
            [qSol, solInfo]= ik(...
                endEffector, sample_poses(:,:,j), weights, ...
                RM.voxRICenterSol{ii});

            ExitFlags(j) = solInfo.ExitFlag;
            solsFound{j} = qSol;

        end

        % store results
        RM.voxRIPoses{ii} = sample_poses;
        RM.voxIKSuccess{ii} = ExitFlags == 1;
        RM.voxIKSolutions{ii} = solsFound;

        time_taken(ii) = toc(t_s);
    end

%      jj=sum(RM.voxValid(1:ii));
%      ssm=ss-jj;
%      ang_calc=t/(jj-9308);
%      [h, m, s] = hms(seconds(ang_calc*ssm));
%    disp([jj / ss, h, m, s])

end
save(params.trial_name+"_complete2_ubuntu.mat","RM");

%%
RM.LocalRIPoses = InverseReachability.zacharias(params.gridres / 2., params.sample_n);

%% Computing RI Normal zacharias
RM.voxRI = InverseReachability.ri_zacharias(RM);
RM.voxRIz= InverseReachability.ri_zacharias(RM);
RM.voxRIzpr= InverseReachability.ri_zachpr(RM);
RM.voxRIzprnn= InverseReachability.ri_zachprnn(RM);


%% Computing RI for task
%
% q=tform2quat(TForm.vec2tform(RM.LocalRIPoses));
% tq=tform2quat(TForm.DOWN);
% z=quatmultiply(quatconj(q),tq);
% a = 2*acosd(z(:,4));
% mask=a<60;
% sum(mask)


TT=TForm.vec2tform(RM.LocalRIPoses);
mask=acosd(min(1,squeeze(abs(TT(3,3,:)))))<60 & squeeze(TT(3,3,:)<0);
sum(mask)

sample_poses=RM.LocalRIPoses(mask,:);


% RM.voxRIRT= vertcat(cellfun(@(c) 100*sum(c)/params.sample_n ,RM.voxIKSuccess ));

%% Visualise poses
if true
%     sample_poses = datasample(poses, 500, 'Replace', false);
    %     sample_poses = datasample(reachable_positions, 500, 'Replace', false);
    %     sample_poses = TForm.tform2vec(poses);
    %     sample_poses = TForm.tform2vec(poses);
    Visualization.draw_poses(sample_poses, 0.05, [1 0 0], 'r');
    hold on
    Visualization.draw_poses(sample_poses, 0.05, [0 1 0], 'g');
    Visualization.draw_poses(sample_poses, 0.05, [0 0 1], 'b');
    hold off
%     ros_helpers.send_rviz_poses(sample_poses, "base_footprint")
end




%% Gererate poses and compute IK
sampling_func = @InverseReachability.PoseSampling.sample_cart_sphere_4dof;
poses = sampling_func(params.gridrad,params.gridres,params.sample_n);
% poses=poses(abs(poses(:,3)-(-0.0750))<0.01,:);
% poses=poses+repmat([0.167 0 0.142 zeros(1,4)],size(poses,1),1);

%%
ik_exists_vec = target_vec(answers == 1, :);

%% %%%%%% Voxelise into IRM %%%%%%
poses = ik_result.ik_exists_vec;
RM = TForm.vec2tform(poses);
IRM = TForm.tform2inv(RM);
iposes = TForm.tform2vec(IRM);

%Voxelise
VoxIRM = voxelise(iposes, params.samplingbias_vox_size, horzcat(ik_result.sols_found{:})'); %xyzrpy
VoxRM = voxelise(poses, params.samplingbias_vox_size, horzcat(ik_result.sols_found{:})'); %xyzrpy
experiment.VoxIRM = VoxIRM;
experiment.VoxRM = VoxRM;
save(params.trial_name + "_exp", "experiment");

%%
% x_points = (1:10)';
% y_points = (1:10)';
% scatter(x_points, y_points);
