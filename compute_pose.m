function pose = compute_pose(data,ctr)
    pose = [0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]; 
    if data(ctr).is_ready == 1
        if length(data(ctr).id) > 0
            pose = process_tag(data,ctr);
        end
        
    end 
end