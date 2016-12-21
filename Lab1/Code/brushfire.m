function [value_map]=brushfire(map)
%this function is based on 8-neighbours relationship
counter=1;
total=size(map,1)*size(map,2)-nnz(map==counter); %% nnz() function counts the number of nonzero elements
while(total>0) % the idea is to subtract the checked cells from the total number
    for i=1:size(map,1)
        for j=1:size(map,2)
            if(map(i,j)==counter)
                n=GetNeighbors(i,j,size(map));
                for a=1:length(n)
                    in1=ceil(n(a)/size(map,2));  %calculating index i
                    in2=n(a)-(in1-1)*size(map,2);   %calcuating index j
                    if(map(in1,in2)==0)
                        map(in1,in2)=map(i,j)+1;
                    end
                end
            end
        end
    end
    counter=counter+1;
    total=total-nnz(map==counter);
end
value_map=map;
end
