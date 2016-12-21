function [value_map, trajectory]=wavefront(map,start,goal)

map(goal(1),goal(2))=2;
n=GetNeighbors(goal(1),goal(2),size(map)); % looking initially for the goal cell neighbors
for a=1:length(n)
    in1=ceil(n(a)/size(map,2));  %calculating index i
    in2=n(a)-(in1-1)*size(map,2);   %calcuating index j
    if(map(in1,in2)==0)
        map(in1,in2)=3;
    end
end
counter=3;
total=size(map,1)*size(map,2)-nnz(map~=0)-1; 
while(total>0) % the idea is to subtract the checked cells from the total number
    for i=1:size(map,1)
        for j=1:size(map,2)
            if(map(i,j)==counter)
                n=GetNeighbors(i,j,size(map));
                for a=1:length(n)
                    in1=ceil(n(a)/size(map,2));  %calculating index i
                    in2=n(a)-(in1-1)*size(map,2);   %calcuating index j
                    if(map(in1,in2)==0)
                        map(in1,in2)=counter+1;
                        total=total-1;
                    end
                end
            end
            
        end
    end
    counter=counter+1;
end
value_map=map;

% second step: finding the trajectory
trajectory=start;%zeros(size(map,1)+size(map,2),2); %calculating the trajectory
i=start(1);j=start(2);
c=2;
while(1)
    if(map(i,j+1)<map(i,j) && map(i,j+1)~=1) % go right
        j=j+1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
        
    elseif(map(i,j-1)<map(i,j)&& map(i,j-1)~=1) % go left
        j=j-1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
        
    elseif(map(i+1,j)<map(i,j)&& map(i+1,j)~=1) % go up
        i=i+1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
        
    elseif(map(i-1,j)<map(i,j)&& map(i-1,j)~=1) % go down
        i=i-1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
    elseif(map(i-1,j+1)<map(i,j)&& map(i-1,j+1)~=1) 
        j=j+1;
        i=i-1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
    elseif(map(i-1,j-1)<map(i,j)&& map(i-1,j-1)~=1)
        j=j-1;
        i=i-1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
    elseif(map(i+1,j-1)<map(i,j)&& map(i+1,j-1)~=1)
        j=j-1;
        i=i+1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
    elseif(map(i+1,j+1)<map(i,j)&& map(i+1,j+1)~=1)
        j=j+1;
        i=i+1;
        trajectory(c,:)=[i j];
        c=c+1;
        if(map(i,j)==map(goal(1),goal(2)))
            break;
        end
    end
end



% second method of implementatin for finding the trajectory. It depends on using a queue 

% count=2;
% neigh=ones(1,8)*(max(size(map,1),size(map,2))+10);
% indeces=zeros(8,2);
% while(1)
%     n=GetNeighbors(start(1),start(2),size(map)); % the neighbours in this function are given in away that they are inserted in up,down,left,right and then diagonal elements priority, which will prioritize these cells to be chosen by the trajectory first
%     for i=1:length(n)
%         ind1=ceil(n(i)/size(map,2));  %calculating index i
%         ind2=n(i)-(ind1-1)*size(map,2);   %calcuating index j
%         neigh(i)=map(ind1,ind2);
%         indeces(i,:)=[ind1 ind2];
%         if(neigh(i)==1)
%             neigh(i)=max(neigh);
%         end
%     end
%     [value,num]=min(neigh);
%     trajectory(count,:)=indeces(num,:); 
%     count=count+1;
%     start=indeces(num,:);
%     if(value==map(goal(1),goal(2)))
%         break;
%     end
% end
% % 



end