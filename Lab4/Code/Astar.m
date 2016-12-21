function [path,minCost]=Astar(vertices, edges)
    n=size(vertices,1); % number of vertices
    heuristic_cost_estimate=zeros(n,1);
    for i=2:size(vertices,1)
        heuristic_cost_estimate(i)=sqrt((vertices(i,1)-vertices(end,1))^2+(vertices(i,2)-vertices(end,2))^2);
    end
    start=1;
    goal=n;
    closedset=[];
    openset=start;
    came_from=zeros(n,1);
    g_score=zeros(n,1);
    f_score=ones(n,1)*Inf;
    f_score(start)=g_score(start)+heuristic_cost_estimate(start);
    while(nnz(openset)~=0)
        op=openset(openset~=0);
        [~,ind]=min(f_score(op));
        current=op(ind);
        if(current==goal)
            path= reconstruct_path(came_from,goal);
            minCost=f_score(goal);
            return;
        end
        openset(openset==current)=0;
        closedset=[closedset current];
        neigh=find(edges(:,1)==current);
        for i=1:length(neigh)
            tentative_g_score = g_score(current) + sqrt((vertices(current,1)-vertices(edges(neigh(i),2),1))^2+(vertices(current,2)-vertices(edges(neigh(i),2),2))^2);
            if (nnz(closedset==edges(neigh(i),2)))
                if(tentative_g_score >= g_score(edges(neigh(i),2)))
                    continue
                end
            end
            if (~nnz(openset==edges(neigh(i),2))|| tentative_g_score < g_score(edges(neigh(i),2)))
                came_from(edges(neigh(i),2))= current;
                g_score(edges(neigh(i),2))=tentative_g_score;
                f_score(edges(neigh(i),2))= g_score(edges(neigh(i),2)) + heuristic_cost_estimate(edges(neigh(i),2));
                if (~nnz(edges(neigh(i),2)==openset))
                    openset=[openset edges(neigh(i),2)];
                end
            end
        end

    end
end


function [path]=reconstruct_path(came_from, current_node)
    if (came_from(current_node))
        p= reconstruct_path(came_from, came_from(current_node));
        path=[p , current_node];
    else
        path=current_node;
    end
end










