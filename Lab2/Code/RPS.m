function [edges]=RPS(vertices)
 
 % step 1
OE=zeros(size(vertices,1)-2,3);  % obstacles' edges
c=2; % index of the first vertix of the poly_num polygon, poly_num=1 initially. it helps to connect the last edge in a ploygon back to the first one
poly_num=1; % polygon number
% loop for building obstacles' edges list and plotting the polygons
for i=1:size(vertices,1)
    if(i<size(vertices,1)-1)
        if(vertices(i+2,3)==poly_num)
            OE(i,1:2)=[i+1 i+2];
        else
            poly_num=poly_num+1;
            OE(i,1:2)=[i+1 c];
            c=i+2; % first vertix of the second polygon
        end
        line([vertices(OE(i,1),1),vertices(OE(i,2),1)],[vertices(OE(i,1),2),vertices(OE(i,2),2)]);hold on;
    end
    text(vertices(i,1),vertices(i,2),horzcat('  ',num2str(i)));
end
scatter([vertices(1,1) vertices(end,1)],[vertices(1,2) vertices(end,2)]);
title('one possible configuration for testing plane sweep algorithm');  

%%%%%%% step 2  %%%%%%
count=1; % counter for visibility graph array
for r=1:size(vertices,1)
    start=[vertices(r,1) vertices(r,2) vertices(r,3) r];  % every 1 would be replaced by i later
    ang=zeros(1,size(vertices,1)-1);
    s=[];%cell(1,size(vertices,1)-2); % unsorted list of edges that intersect the horizontal half-line
    j=1;
    % loop for building the alpha and S list and computing the distance between the
    % center of each edge and the start point 
    for i=1:size(vertices,1)
        ang(1,i)=(atan2(vertices(i,2)-start(2),vertices(i,1)-start(1)));
        if(i<size(vertices,1)-1) 
            cent_point= [ ( vertices(OE(i,1),1)+vertices(OE(i,2),1) )/2 ,  ( vertices(OE(i,1),2)+vertices(OE(i,2),2) )/2  ];
            text(cent_point(1,1),cent_point(1,2),horzcat('Edge  ',num2str(i))); % show the edges' names on the current figure
            OE(i,3)=norm(cent_point-start(1,1:2));  % euclidean distance between the center of the edge i-1 and the start point
            if(vertices(OE(i,1),2)<start(1,2) && vertices(OE(i,2),2)>start(1,2)&& vertices(OE(i,2),1)>start(1,1)|| vertices(OE(i,1),2)>start(1,2) && vertices(OE(i,2),2)<start(1,2)&&vertices(OE(i,1),1)>start(1,1) )
                s(j)=i;
                j=j+1;
            end
        end        
        if(ang(1,i)<0)
            ang(1,i)=ang(1,i)+2*pi;
        end
    end
    [y,alpha]=sort(ang);
    alpha(1)=[];
    [~,temp2]=sort(OE(s,3));
    s=s(temp2);
    
    %%%%  step 3 finding the visible node/s to the start node %%%%%
   
    for i=1:length(alpha)
        
        %%%%%%%%%%%%%%%%%%%%    checking visibility of the current node       %%%%%%%%%%%%%%%%%%%%
        if(~isempty(s))  % determin if the current vertix ,which is under study, is visible or not
            if(alpha(i)==OE(s(1),1) || alpha(i)==OE(s(1),2)|| ( norm( vertices(alpha(i),1:2)-start(1,1:2) ) <OE(s(1),3)  )) %  (vertices(OE(s(1),1),3)~=vertices(start(1,4),3))if yes, then it is visible. OE(sorted_s(i),1): the first vertix of edge i in the sorted list of edges.% if this vertix is closer than the closet edge in the sorted list, then it is also visible
                if( ( OE(OE(:,1)==alpha(i),2)==start(1,4) ||  OE(OE(:,2)==alpha(i),1)==start(1,4) )    ||  (  vertices(alpha(i),3)~=start(1,3) && isempty(find(vertices(OE(s(1,:),1),3)==vertices(start(1,4),3)) )))
                    VG(count,:)=[start(1,4)  alpha(i)];
                    count=count+1;
                end
            end
        elseif(isempty(s))
           VG(count,:)=[start(1,4) alpha(i)];
           count=count+1;

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%    updating S list               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        fir_edg=find(OE(:,1)==alpha(i));  % the index of the edge that has vertix alpha(i) as the first vertix
        sec_edg=find(OE(:,2)==alpha(i)); % the index of the edge that has vertix alpha(i) as the second vertix
        fir_ind=[];sec_ind=[];
        if(~isempty(fir_edg)) % enough to check one of the edges because each vertix has either two edges or non, when it is the start or end node
            fir_ind=find(s==fir_edg); 
            sec_ind=find(s==sec_edg);
            if( isempty(fir_ind) && OE(fir_edg,1)~=start(1,4) && OE(fir_edg,2)~=start(1,4))  % update the list of edges
                s(j)=fir_edg;
                j=j+1;
            elseif(~isempty(fir_ind))
                s(fir_ind)=[];  % delete this element from s
                j=j-1; % becuase deleting element from s changes the indeces of this matrix
                if(sec_ind>fir_ind )
                    sec_ind=sec_ind-1;  % becuase deleting element from s changes the indeces of this matrix
                end
            end
            if(isempty(sec_ind) && OE(sec_edg,1)~=start(1,4) && OE(sec_edg,2)~=start(1,4))
                s(j)=sec_edg;
                j=j+1;
            elseif(~isempty(sec_ind))
                s(sec_ind)=[];
                j=j-1;
            end
            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Sorting S list             %%%%%%%%%%%%%%%%%%%%%%
        [~,temp2]=sort(OE(s,3));  % sort the updated version of the list of edges
        s=s(temp2);
        end
    end
end


 % removing repetition from VG        
for i=1:size(VG,1)
   ind=find(VG(:,1)==VG(i,2));
   for j=1:length(ind)
       if(VG(ind(j),2)==VG(i,1))
           VG(ind(j),:)=[0 0];
       end
   end
end
VG(VG(:,1)==0,:)=[];
edges=VG;

% ploting visible edges 
for i=1:size(VG,1)
    ch1=find(OE(:,1)==VG(i,1));ch2=find(OE(:,1)==VG(i,2));
    if(isempty(ch1)||OE(ch1,2)~=VG(i,2))
        if(isempty(ch2)||OE(ch2,2)~=VG(i,1))
        line([vertices(VG(i,1),1),vertices(VG(i,2),1)],[vertices(VG(i,1),2),vertices(VG(i,2),2)],'color','r');    
        end
    end
end



end

        
        
        
        
        
        
        
        
    


