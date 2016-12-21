function [neigh]=GetNeighbors(i,j,s)
%8-neighbours
if(i==1)
        if(j==1)
            neigh(1)=(i-1)*s(2)+j+1;     %%first neigh
            neigh(2)=((i-1)+1)*s(2)+j+1; %%second neigh
            neigh(3)=((i-1)+1)*s(2)+j;   %%third neigh 
        elseif(j==s(2))
            neigh(1)=(i-1)*s(2)+j-1;     %%first neigh
            neigh(2)=((i-1)+1)*s(2)+j;   %%second neigh 
            neigh(3)=((i-1)+1)*s(2)+j-1; %%third neigh 
        else
            neigh(1)=(i-1)*s(2)+j+1;     %%first neigh
            neigh(2)=((i-1)+1)*s(2)+j;   %%second neigh 
            neigh(3)=(i-1)*s(2)+j-1;     %%third neigh 
            neigh(4)=((i-1)+1)*s(2)+j-1; %%fourth neigh 
            neigh(5)=((i-1)+1)*s(2)+j+1; %%fifth neigh 
        end
                    
elseif(i==s(1))
        if(j==1)
            neigh(1)=((i-1)-1)*s(2)+j;   %%first neigh
            neigh(2)=(i-1)*s(2)+j+1;     %%second neigh 
            neigh(3)=((i-1)-1)*s(2)+j+1; %%third neigh 
       elseif(j==s(2))
            neigh(1)=((i-1)-1)*s(2)+j;   %%first neigh
            neigh(2)=(i-1)*s(2)+j-1;     %%second neigh
            neigh(3)=((i-1)-1)*s(2)+j-1; %%third neigh 
        else
            neigh(1)=(i-1)*s(2)+j+1;     %%first neigh
            neigh(2)=((i-1)-1)*s(2)+j;   %%second neigh 
            neigh(3)=(i-1)*s(2)+j-1;     %%third neigh 
            neigh(4)=((i-1)-1)*s(2)+j-1; %%fourth neigh
            neigh(5)=((i-1)-1)*s(2)+j+1; %%fifth neigh 
        end  
elseif(j==1)
        neigh(1)=((i-1)-1)*s(2)+j;      %%first neigh
        neigh(2)=((i-1)+1)*s(2)+j;      %%second neigh 
        neigh(3)=(i-1)*s(2)+j+1;        %%third neigh 
        neigh(4)=((i-1)+1)*s(2)+j+1;    %%fourth neigh 
        neigh(5)=((i-1)-1)*s(2)+j+1;    %%fifth neigh 
elseif(j==s(2))
        neigh(1)=((i-1)-1)*s(2)+j;      %%first neigh
        neigh(2)=((i-1)+1)*s(2)+j;      %%second neigh 
        neigh(3)=(i-1)*s(2)+j-1;        %%third neigh
        neigh(4)=((i-1)+1)*s(2)+j-1;    %%fourth neigh
        neigh(5)=((i-1)-1)*s(2)+j+1;    %%fifth neigh 
else
       neigh(1)=((i-1)-1)*s(2)+j;       %%first neigh
       neigh(2)=((i-1)+1)*s(2)+j;       %%second neigh 
       neigh(3)=(i-1)*s(2)+j-1;         %%third neigh  
       neigh(4)=(i-1)*s(2)+j+1;         %%fourth neigh 
       neigh(5)=((i-1)-1)*s(2)+j-1;     %%fifth neigh 
       neigh(6)=((i-1)-1)*s(2)+j+1;     %%sixth neigh 
       neigh(7)=((i-1)+1)*s(2)+j-1;     %%seventh neigh 
       neigh(8)=((i-1)+1)*s(2)+j+1;     %%eighth neigh  
    
end

end