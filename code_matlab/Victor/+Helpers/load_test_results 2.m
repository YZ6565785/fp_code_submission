function [test_para, data_loaded] = load_test_results(type)
    test_para = [];
    file_path = "Victor/data/path_saved/";

    file_name = "";
    files = dir(file_path);
    file_names = {files.name};
    sort(file_names)
    data_loaded = [];
    ind = 1;

    result_file_names = {};
    for i=1:length(file_names)
       if contains(file_names{i},"obs_path_results_"+type)
            name_fields = split(file_names{i},'_');
            para = str2double(extractBefore(name_fields{end},'.mat'));
            test_para = [test_para; para];
            result_file_names{end+1,1} = file_names{i};
       end
    end
    
    [~,I] = sort(test_para); % sort by parameter value
    result_file_names = result_file_names(I);
    for i=1:size(result_file_names,1)
        name_fields = split(result_file_names{i},'_');
        disp("loading... "+ result_file_names{i});
        disp("Hold on.")
        file_path+result_file_names{i}
        load(file_path+result_file_names{i},"test_results");
        data_loaded{ind} = test_results;
        ind = ind + 1;
    end
    disp("Completed!")
    data_loaded
end
