function [path_smooth]=smooth(map,path,vertices,delta)
% smoothig function
flip_path=flip(path);
v_end=vertices(flip_path(end),:);% v_end initially equals the goal point
v_end_index=flip_path(end);
path_smooth=flip_path(end); % path_smooth initially has the goal point index
while(path_smooth(end)~=1) % loop for generating the smoothed path
    i=1;
    while(i<v_end_index) % loop for finding a connectable path
        v_start=vertices(flip_path(i),:);
        belong_to_obstacle=0;
        for j=1:ceil(norm([v_start(1)-v_end(1),v_start(2)-v_end(2)])/delta) % incremental checking for the line connecting two nodes
              f=@(theta)([cos(theta) -sin(theta) v_start(1); sin(theta) cos(theta) v_start(2)]); % transformation matrix
              xy=round(f(atan2(v_end(2)-v_start(2),v_end(1)-v_start(1)))*[j*delta;0;1]) ;
              xy=xy';
              if(map(xy(2),xy(1))==1) % if the path passes through an obstacle
                  belong_to_obstacle=1;
                  break;
              end
        end
        if(belong_to_obstacle==0) % if the line lies entirely in the free space
            path_smooth=[path_smooth flip_path(i)];
            break;
        else
            i=i+1;

        end
    end
    % move to a nearer vertex to the start
    v_end_index=flip_path(i);
    v_end=vertices(flip_path(i),:);
end

end




