% Script for testing all the given maps using Brushfire algorithm

% 1- script testing the small map
clear all;clc;
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

[value_map]=brushfire(map);
pl=[map map(:,end)];
pl=[pl;pl(end,:)];
pcolor(pl);axis ij; axis square;
title('read color represents the obstacles and other colors are empty cells');figure;
pl=[value_map value_map(:,end)];
pl=[pl;pl(end,:)];
pcolor(pl);axis ij; axis square;
title('Dark blue color represents the obstacles and every other color represents different weight');

%% 2- script for testing maze.m.mat file

load('maze.m.mat')
[value_map]=brushfire(map);
cmap=colormap;
imwrite(map,'original_map_maze.png');
imwrite(value_map,cmap,'brushfire_maze.jpeg');
a=imread('original_map_maze.png');
figure;imshow(a);title('Maze map');
b=imread('brushfire_maze.jpeg');
figure;imshow(b);title('brushfire of Maze map');

%% 3- script for testing mazeBig.m.mat file

load('mazeBig.m.mat')
[value_map]=brushfire(map);
cmap=colormap;
imwrite(map,'original_map_mazeBig.png');
imwrite(value_map,cmap,'brushfire_mazeBig.jpeg');
a=imread('original_map_mazeBig.png');
figure;imshow(a);title('MazeBig map');
b=imread('brushfire_mazeBig.jpeg');
figure;imshow(b);title('brushfire of MazeBig');

%% 4- script for testing obstaclesBig.m.mat file

load('obstaclesBig.m.mat')
[value_map]=brushfire(map);
cmap=colormap;
imwrite(map,'original_map_obstaclesBig.png');
imwrite(value_map,cmap,'brushfire_obstaclesBig.jpeg');
a=imread('original_map_obstaclesBig.png');
figure;imshow(a);title('obstaclesBig map');
b=imread('brushfire_obstaclesBig.jpeg');
figure;imshow(b);title('brushfire of obstaclesBig map');






