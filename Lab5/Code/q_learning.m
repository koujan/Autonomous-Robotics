function [Q]=q_learning(map,q_goal,alpha,gamma,epsilon,n_episodes,n_iterations)

pl=[map map(:,end)];
effec=zeros(1,n_episodes/100);
pl=[pl;pl(end,:)];
pcolor(pl);axis ij; axis square;
title('read color represents the obstacles and other colors are empty cells');close;
[map_y,map_x]=size(map);
Q=zeros(map_y,map_x,4);
right=1; left=2; up=3; down=4;
k=1;
move=[1,0;-1,0;0,-1;0,1];
s=zeros(1,2);
for i=1:n_episodes
    s(1,1)=randi(map_x);
    s(1,2)=randi(map_y);
    while( map(s(2),s(1))==1 || isequal(s,q_goal))
        s(1,1)=randi(map_x);
        s(1,2)=randi(map_y);
    end
    for j=1:n_iterations
        if(rand()>epsilon)
            [~,a]=max(Q(s(2),s(1),:));
        else
            a=randi(4);
        end
        s2=s+move(a,:);
        if(map(s2(2),s2(1))==1)
            s2=s;
            r=-1;
        elseif(isequal(s2,q_goal))
            r=1;
        else
            r=-1;
        end
        Q(s(2),s(1),a)=Q(s(2),s(1),a)+alpha*(r+gamma*max(Q(s2(2),s2(1),:))-Q(s(2),s(1),a));
        s=s2;
        if(isequal(s2,q_goal))
            break
        end   
    end
    % Computing the effectiveness
    if (i==k*100)
        cum_r=0;
        k=k+1;
        for c=1:100
            s(1,1)=randi(map_x);
            s(1,2)=randi(map_y);
            while( map(s(2),s(1))==1 || isequal(s,q_goal))
                s(1,1)=randi(map_x);
                s(1,2)=randi(map_y);
            end
            for u=1:n_iterations
                [~,a]=max(Q(s(2),s(1),:));
                s2=s+move(a,:);
                if(map(s2(2),s2(1))==1)
                    s2=s;
                    cum_r=cum_r-1;
                elseif(isequal(s2,q_goal))
                    cum_r=cum_r+1;
                else
                    cum_r=cum_r-1;
                end
                s=s2;
            end
        end
        effec(k-1)=cum_r/(100);
    end      
end
% Constructing the state value function and the optimal policy
val=zeros(size(map));
figure;
step=0.01;
step2=step*1.5;
norm=21;
shift=size(map,1)+4;
for i=1:size(map,1)
    for j=1:size(map,2)
        
        [val(i,j),ind]=max(Q(i,j,:));
        if(val(i,j)~=0)
            switch ind
                case 1
                    annotation('arrow',[j/norm-step j/norm+step],[(shift-i)/norm,(shift-i)/norm],'Color','r');
                case 2
                    annotation('arrow',[j/norm+step j/norm-step],[(shift-i)/norm,(shift-i)/norm],'Color','r');
                case 3
                    annotation('arrow',[j/norm j/norm],[(shift-i)/norm-step2,(shift-i)/norm+step2],'Color','r');
                case 4
                    annotation('arrow',[j/norm j/norm],[(shift-i)/norm+step2,(shift-i)/norm-step2],'Color','r');
            end
        else
            if(isequal([j,i],q_goal)) 
                % highlight the goal
                annotation('rectangle',[j/norm-step ,(shift-i)/norm-step, 2*step,2*step],'facecolor',[0 1 0]);
            else
                annotation('rectangle',[j/norm-step ,(shift-i)/norm-step, 2*step,2*step],'facecolor',[0 0 0]);
            end
        end
    end
end
% Graphical information for the effectiveness and state value function
figure;
pl=[val val(:,end)];
pl=[pl;pl(end,:)];
pcolor(pl);axis ij; axis square;
title('State Value Function V');
figure;
plot(100:100:n_episodes,effec);
title('Evolution of the Effectiveness over Episodes');
end
