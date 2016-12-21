% Run & draw file to test the smooth and RRT functions

clear all; clc;
% for first map
load('map.mat');
q_start=[80,70];  q_goal=[707,615]; % map
%q_start=[424,350];  q_goal=[175,555]; % map

% for second map
%load('maze.mat');
%q_start=[206, 198]; q_goal=[416, 612]; % maze
%q_start=[25, 25]; q_goal=[360,548];  % maze (test)

% input parameters
k=10000;delta_q=50 ; p=0.3;delta=5;
[vertices,edges,path]=RRT(map,q_start,q_goal,k,delta_q,p);
[path_smooth]=smooth(map,path,vertices,delta);
colormap=[1 1 1; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
imshow(uint8(map),colormap);
hold on;
text(vertices(1,1),vertices(1,2),'start   ');
text(vertices(end,1),vertices(end,2),'  goal');

% draw the whole tree of vertices
for i=1:length(edges)
    v1=vertices(edges(i,1),:);
    v2=vertices(edges(i,2),:);
    line([v1(1),v2(1)],[v1(2),v2(2)]);
    plot(v1(1),v1(2),'*g');
    %text(v1(1),v1(2),[' ' num2str(edges(i,1))]);
    %text(v2(1),v2(2),[' ' num2str(edges(i,1))]);
end

% draw the entire path
for i=1:length(path)-1
    v1=vertices(path(i),:);
    v2=vertices(path(i+1),:);
    line([v1(1),v2(1)],[v1(2),v2(2)],'color','r');
end

% draw the smoothed path
for i=1:length(path_smooth)-1
    v1=vertices(path_smooth(i),:);
    v2=vertices(path_smooth(i+1),:);
    line([v1(1),v2(1)],[v1(2),v2(2)],'color','k');
end