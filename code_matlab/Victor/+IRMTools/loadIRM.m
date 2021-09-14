function irm = loadIRM(res_IRM, robot_name, num_poses)
    if ~exist('num_poses','var')
       num_poses = 50; 
    end

    IRM_path = "Victor/data/irm_saved/";
        
    IRM_file_name = "";
    files = dir(IRM_path);
    file_names = {files.name};
    sort(file_names);
    for i=length(file_names):-1:1
       if contains(file_names{i},"IRM_"+robot_name+"_r"+res_IRM+"_"+num_poses+"poses")
           IRM_file_name = file_names{i};
           break;
       end
    end
    
    assert(IRM_file_name~="", "No IRM for "+robot_name+" for res=" + ...
        res_IRM +" saved before."+newline+'Check if IRM has been saved before!');
    disp(IRM_file_name);
    load(IRM_path+IRM_file_name);
    disp("*** IRM loaded with " + size(irm.map,1) + " maps.");
    
end