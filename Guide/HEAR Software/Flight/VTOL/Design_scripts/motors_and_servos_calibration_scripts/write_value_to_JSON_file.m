function write_value_to_JSON_file(json_file_path, name, value)
    % Write 'value' that corresponds to the given 'name'
    % 'name', 'value' pair will be appended if 'name' does not exist

    if isfile(json_file_path) % Read from a file if it exists
        read_json_txt = fileread(json_file_path);
        b_struct = jsondecode(read_json_txt);
    else
        % Initialize the file with an empty struct
        b_struct = struct();
    end

    if isfield(b_struct, name)
        disp(['Changing->', name])
    else
        disp(['Creating->', name])
    end

    % Add or update the 'name' field in the struct
    b_struct.(name) = value;

    % Convert the struct to JSON and write to the file
    b = jsonencode(b_struct, 'PrettyPrint', true);
    write_to_file(json_file_path, b);
end

function write_to_file(json_file_path, data)
    fid = fopen(json_file_path, 'w');
    if fid == -1
        error(['File could not be opened for writing: ', json_file_path]);
    end
    fwrite(fid, data, 'char');
    fclose(fid);
end

