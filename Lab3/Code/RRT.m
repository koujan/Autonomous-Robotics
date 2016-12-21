function [vertices,edges,path]=RRT(map,q_start,q_goal,k,delta_q,p)

% Initialize the “vertices” variable with q_start
vertices=q_start;
ver_index=2;
% For k samples repeat
for i=1:k
   r=rand;
   if(r<p) % With p probability use q_rand = q_goal
        q_rand=q_goal;
   else    % Otherwise generate q_rand in the dimensions of the map
        q_rand=[rand*size(map,2) ,rand*size(map,1)];
        q_rand=ceil(q_rand);
   end
   % Find q_near from q_rand in “vertices”
   dist=sqrt((vertices(:,1)-q_rand(1)).^2+(vertices(:,2)-q_rand(2)).^2);
   [~,ind]=min(dist);
   q_near=vertices(ind,:);
   % generating q_new at delta_q distance from q_near in the direction to q_rand.
   f=@(theta)([cos(theta) -sin(theta) q_near(1); sin(theta) cos(theta) q_near(2)]); %transformation matrix
   q_new=round (f(atan2(q_rand(2)-q_near(2),q_rand(1)-q_near(1) ) )*[delta_q;0;1]);
   q_new=q_new';
   if(norm([q_new(1)-q_near(1),q_new(2)-q_near(2)])>norm([q_rand(1)-q_near(1),q_rand(2)-q_near(2)])) % in case the q_new goes beyond the q_rand
       q_new=q_rand;
   end
   % If q_new belongs to free space
   if(map(q_new(2),q_new(1))~=1)
       % If the edge between q_near and q_new belongs to free space
       belong_to_obstacle=0;
       step=norm([q_new(1)-q_near(1),q_new(2)-q_near(2)])/11; % step size between each of the 10 points on the current edge
       for j=1:10
          f_incremental=@(theta)([cos(theta) -sin(theta) q_near(1); sin(theta) cos(theta) q_near(2)]);
          xy=round(f_incremental(atan2(q_new(2)-q_near(2),q_new(1)-q_near(1)))*[j*step;0;1]) ;
          xy=xy';
          %x=round(q_near(1)+cos(atan2(q_new(2)-q_near(2),q_new(1)-q_near(1)))*j*step);
          %y=round(q_near(2)+sin(atan2(q_new(2)-q_near(2),q_new(1)-q_near(1)))*j*step); 
          if(map(xy(2),xy(1))==1)
              belong_to_obstacle=1;
              break;
          end
       end
       if(~belong_to_obstacle)
           % Add q_new in “vertices”
           vertices(ver_index,:)=q_new;
           % Add [index(q_new) index(q_near)] in “edges”
           edges(ver_index-1,:)=[ver_index ind];
           ver_index=ver_index+1;
           if(q_new==q_goal)
               % Fill “path” and stop RRT function
               path=edges(end,:);
               c=edges(end,2);    
               while(c~=1)
                   path=[path edges(c-1,2)];
                   c=edges(c-1,2);
               end
               return;
               
           end
           
       end
       
   end
   
end
 disp('Not enough samples to reach the goal');
 path=edges(end,:);
 c=edges(end,2);    
 while(c~=1)
    path=[path edges(c-1,2)];
    c=edges(c-1,2);
 end
 
 
end