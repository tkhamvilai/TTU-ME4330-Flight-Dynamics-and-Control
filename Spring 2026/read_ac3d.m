function [vertices, faces] = read_ac3d(filename)
    % READ_AC3D Simple parser for AC3D files
    % Reads all vertices and poly refs combined into one list.
    
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file.');
    end
    
    vertices = [];
    faces = {}; % Use cell array as faces might have different vertex counts
    
    current_verts = [];
    global_vert_offset = 0;

    while ~feof(fid)
        line = strtrim(fgetl(fid));
        
        % Parse Vertices
        if startsWith(line, 'numvert')
            parts = sscanf(line, 'numvert %d');
            num_verts = parts(1);
            
            % Read the vertex lines
            new_verts = fscanf(fid, '%f %f %f', [3, num_verts])';
            
            % Store vertices (accumulate if multiple objects)
            current_verts = new_verts; 
            vertices = [vertices; new_verts];
        end
        
        % Parse Surfaces (Faces)
        if startsWith(line, 'SURF')
            % The next few lines contain 'mat' and 'refs'
            % We look specifically for 'refs'
            while ~feof(fid)
                subline = strtrim(fgetl(fid));
                if startsWith(subline, 'refs')
                    ref_parts = sscanf(subline, 'refs %d');
                    num_refs = ref_parts(1);
                    
                    % Read vertex indices for this face
                    % AC3D indices are 0-based; MATLAB is 1-based.
                    % We must add the global offset to align with accumulated vertices.
                    v_idxs = fscanf(fid, '%d', num_refs)' + 1 + global_vert_offset;
                    
                    faces{end+1, 1} = v_idxs;
                    break; % Done with this surface
                end
            end
        end
        
        % Handle Object separation (update offset)
        if startsWith(line, 'kids')
            % If we finish an object, the next 'numvert' block will rely 
            % on a new index set, but if we are merging them, we need to
            % track the total count.
            global_vert_offset = size(vertices, 1);
        end
    end
    fclose(fid);
end