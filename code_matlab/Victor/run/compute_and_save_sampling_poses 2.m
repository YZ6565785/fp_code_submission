num_poses = 200
res = 0.05
samples = InverseReachability.zacharias(res, num_poses);
size(samples)
csvwrite("../reuleaux_fp/map_creator/maps/sample_poses_"+num_poses+".csv", samples);
