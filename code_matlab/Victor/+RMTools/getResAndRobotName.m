function [res, robot_name] = getResAndRobotName(bl_file_name)

    parts = split(bl_file_name, "_");
    parts = convertCharsToStrings(parts);
    robot_name = parts(2);
    i = 3;
    while extract(parts(i),1) ~= 'r'
        robot_name = robot_name + "_" + parts(i);
        i = i + 1;
    end
    res = str2double(extractAfter(parts(i),2));

end