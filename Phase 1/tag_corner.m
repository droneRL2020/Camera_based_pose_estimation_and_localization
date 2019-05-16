function [x,y,x_c,y_c] = tag_corner(data,ctr,x1,y1,tag,i)
    if tag == "p0"
        x = x1 + 0.152/2;
        y = y1 + 0.152/2;
        world_cord = data(ctr).p0(:,i);
    end
    if tag == "p1"
        x = x1 + 0.152;
        y = y1;
        world_cord = data(ctr).p1(:,i);
    end
    if tag == "p2"
        x = x1 + 0.152;
        y = y1 + 0.152;
        world_cord = data(ctr).p2(:,i);
    end
    if tag == "p3"
        x = x1;
        y = y1 + 0.152;
        world_cord = data(ctr).p3(:,i);
    end
    if tag == "p4"
        x = x1;
        y = y1;
        world_cord = data(ctr).p4(:,i);
    end
    x_c = world_cord(1);
    y_c = world_cord(2);
end