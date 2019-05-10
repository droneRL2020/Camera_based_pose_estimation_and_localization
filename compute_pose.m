function rpy = compute_pose(data,ctr)
    rpy = [0 0 0 0]; 
    if data(ctr).is_ready == 1
        if length(data(ctr).id) > 0
            rpy = process_tag(data,ctr);
        end
        
    end 
end