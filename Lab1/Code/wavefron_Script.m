% Script for testing all the given maps using Wavefront algorithm

% 1- script testing the small map
clear all;clc;
goal=[3 18];
start=[13 2];
map=[
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;
1 0 0 0 0 0 1 1 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 1 1 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 1 1 1 1 1 0 0 0 0 0 1 1 0 0 0 1;
1 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 1;
1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1;
1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1;
1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1;
1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1;];

[value_map,trajectory]=wavefront(map,start,goal);
pl=[value_map value_map(:,end)];
pl=[pl;pl(end,:)]; % the aim of this duplicatoin is that in pcolor(c) the last row and column of c are not used
pcolor(pl);axis ij; axis square;title('Dark blue color represents obstacles');
%figure,pcolor(map);hold on; 
for i=1:length(trajectory)
     map(trajectory(i,1),trajectory(i,2))=10;
end
pl=[map map(:,end)];
pl=[pl;pl(end,:)];
pcolor(pl);axis ij,axis square;
title('map optimum trajectory');


%% 2- script for testing maze.m.mat file

load('maze.m.mat')
goal=[5 150];
start=[45 4];
[value_map,trajectory]=wavefront(map,start,goal);
cmp=winter; 
imwrite(value_map,cmp,'maze value map.jpg');
im=imread('maze value map.jpg');
figure;imshow(im);
title('maze value map');

map_mod=10*map+30;
for i=1:length(trajectory)
     map_mod(trajectory(i,1),trajectory(i,2))=10;
end
cmp=winter; 
imwrite(map_mod,cmp,'maze trajectory map.jpg');
im=imread('maze trajectory map.jpg');
figure;imshow(im);
title('maze trajectory map');

%% 3- script for testing mazeBig.m.mat file

load('mazeBig.m.mat')
goal=[460 230];
start=[600 100];
[value_map,trajectory]=wavefront(map,start,goal);
cmp=winter; 
imwrite(value_map,cmp,'mazeBig value map.jpg');
im=imread('mazeBig value map.jpg');
figure;imshow(im);
title('mazeBig value map');

map_mod=10*map+30;
for i=1:length(trajectory)
     map_mod(trajectory(i,1),trajectory(i,2))=10;
end
cmp=winter; 
imwrite(map_mod,cmp,'mazeBig trajectory map.jpg');
im=imread('mazeBig trajectory map.jpg');
figure;imshow(im);
title('mazeBig trajectory map');

%% 4- script for testing obstaclesBig.m.mat file

load('obstaclesBig.m.mat')
goal=[34,45];
start=[567,786];
[value_map,trajectory]=wavefront(map,start,goal);
cmp=colormap; 
imwrite(value_map,cmp,'obstaclesBig value map.jpg');
im=imread('obstaclesBig value map.jpg');
figure;imshow(im);
title('obstaclesBig value map');

map_mod=10*map+30;
for i=1:length(trajectory)
     map_mod(trajectory(i,1),trajectory(i,2))=10;
end
cmp=winter; 
imwrite(map_mod,cmp,'obstaclesBig trajectory map.jpg');
im=imread('obstaclesBig trajectory map.jpg');
figure;imshow(im);
title('obstaclesBig trajectory map');





