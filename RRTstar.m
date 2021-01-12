%2021.1.12修改完成，可绕开两个管状障碍物，并可以画出路径点到障碍物的距离
clear all; close all;
%% 绘制障碍物(圆柱体)
ob1.p1=[0,50,50];
ob1.p2=[100,50,50];
ob1.r=10;%半径
ob2.p1=[80,100,50];
ob2.p2=[80,100,150];
ob2.r=10;
obstacle=[ob1,ob2];
%画出图像
figure(1);
cylinder3(ob1.p1,ob1.p2,ob1.r,50,'y',1,0);
cylinder3(ob2.p1,ob2.p2,ob2.r,50,'y',1,0);
axis equal
%% 定义起点终点
source.coord=[10 10 10];
source.cost=0;
source.parent=0;
[feasible dis]=feasiblePoint3(source.coord, obstacle);
source.dis=dis;	%起点到两个障碍物的距离
goal.coord=[150 150 150];
goal.cost=0;
[feasible dis]=feasiblePoint3(goal.coord, obstacle);
goal.dis=dis;

stepsize = 10; %步长
threshold = 10; %到达目标点周围的阈值
maxFailedAttempts = 10000;  %探索次数
display = true; 
searchSize = [250 250 250];      %探索空间六面体
%% 绘制起点和终点
hold on;
scatter3(source.coord(1),source.coord(2),source.coord(3),"filled","g");
scatter3(goal.coord(1),goal.coord(2),goal.coord(3),"filled","b");
tic;  % tic-toc: Functions for Elapsed Time
RRTree(1) = source;
failedAttempts = 0;
pathFound = false;

%% 循环
while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% 随机生成点
    if rand < 0.5
        sample.coord = rand(1,3) .* searchSize;   % 在搜索范围内随机生成点
    else
        sample = goal; % 以样本为目标，使树生成偏向目标
    end
    %% 找到树内距离随机点最近的节点
    for j = 1:1:length(RRTree)
    ndist(j,1)=distanceCost(RRTree(j).coord,sample.coord);
    end
    [A, I] = min( ndist,[],1); % 找出每一列的最小值
    closestNode = RRTree(I(1));
    %% 将树内节点沿着节点和随机点的方向延长一个stepsize距离生成新点
    movingVec = sample.coord-closestNode.coord;
    movingVec = movingVec/sqrt(sum(movingVec.^2));  %单位化
    newPoint.coord = closestNode.coord + stepsize * movingVec;
    [feasible dis]=checkPath3(closestNode.coord, newPoint.coord, obstacle);
    if ~feasible % 碰撞检测        
        failedAttempts = failedAttempts + 1;
        continue;
    end
    newPoint.dis=dis;
    if distanceCost(newPoint.coord,goal.coord) < threshold, pathFound = true; break; end % 判断是否到达终点附近
    for j = 1:1:length(RRTree)
    [A, I2] = min( distanceCost(RRTree(j).coord,newPoint.coord) ,[],1); % 判断新点是否已经在树内
    end
    if distanceCost(newPoint.coord,RRTree(I2(1)).coord) < threshold, failedAttempts = failedAttempts + 1; continue; end 
    
    newPoint.cost=distanceCost(newPoint.coord,closestNode.coord)+closestNode.cost;  %计算目前的代价
    
    % 寻找树内与newPoint距离小于R的节点
    q_nearest=[];
    R=60;
    neighbor_count=1;
    for j = 1:1:length(RRTree)
            if checkPath3(RRTree(j).coord, newPoint.coord, obstacle) && distanceCost(RRTree(j).coord, newPoint.coord) <= R    %碰撞检测、距离检测
                q_nearest(neighbor_count).coord = RRTree(j).coord;
                q_nearest(neighbor_count).cost = RRTree(j).cost;
                neighbor_count = neighbor_count+1;
            end
    end
    
    % 初始化代价
    q_min=closestNode;C_min=newPoint.cost;
    % 遍历所有临近点（R），寻找最低代价节点
    % path
    for k = 1:1:length(q_nearest)
            if  checkPath3(q_nearest(k).coord, newPoint.coord,obstacle) && q_nearest(k).cost + distanceCost(q_nearest(k).coord, newPoint.coord) < C_min       %碰撞检测、代价最小化
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + distanceCost(q_nearest(k).coord, newPoint.coord);
                     line([q_min.coord(1), newPoint.coord(1)], [q_min.coord(2), newPoint.coord(2)],[q_min.coord(3), newPoint.coord(3)] ,'Color', 'b');        %画出更新父节点的线段        
                     hold on
            end
    end
    
     % 更新父节点
        for j = 1:1:length(RRTree)
            if RRTree(j).coord == q_min.coord
                newPoint.parent = j;
                newPoint.cost=distanceCost(newPoint.coord,RRTree(j).coord)+RRTree(j).cost;
            end
        end
    
    
    RRTree = [RRTree  newPoint]; % 添加到树内
    failedAttempts = 0;  
    if display, plot3([closestNode.coord(1);newPoint.coord(1)],[closestNode.coord(2);newPoint.coord(2)],[closestNode.coord(3);newPoint.coord(3)],'LineWidth',1); end
    pause(0.05);
end

if display && pathFound, plot3([closestNode.coord(1);goal.coord(1)],[closestNode.coord(2);goal.coord(2)],[closestNode.coord(3);goal.coord(3)]); end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end
%% retrieve path from parent information
D = [];
for j = 1:1:length(RRTree)
    tmpdist = distanceCost(RRTree(j).coord, goal.coord);
    D = [D tmpdist];
end

[val, idx] = min(D);
q_final = RRTree(idx);
goal.parent = idx;
q_end = goal;
RRTree = [RRTree goal];

path = q_end;
prev = I(1);
while prev > 0
    path = [RRTree(prev); path];
    prev = RRTree(prev).parent;
end

pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost(path(i).coord,path(i+1).coord); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
figure(2)
cylinder3(ob1.p1,ob1.p2,ob1.r,50,'b',1,0);
cylinder3(ob2.p1,ob2.p2,ob2.r,50,'b',1,0);
axis equal
hold on;
scatter3(source.coord(1),source.coord(2),source.coord(3),"filled","g");
scatter3(goal.coord(1),goal.coord(2),goal.coord(3),"filled","b");
ppath=[];
path_dis=[];
for i=1:1:length(path)
ppath=[ppath;path(i).coord];
path_dis=[path_dis;path(i).dis];
end
plot3(ppath(:,1),ppath(:,2),ppath(:,3),'LineWidth',2,'color','r');
figure(3)
plot(path_dis);
%% checkPath3.m	
function [feasible dis]=checkPath3(n,newPos,obstacle)
feasible=true;
movingVec = [newPos(1)-n(1),newPos(2)-n(2),newPos(3)-n(3)];
movingVec = movingVec/sqrt(sum(movingVec.^2)); %单位化
for R=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n + R .* movingVec;
    if ~(feasiblePoint3(ceil(posCheck),obstacle) && feasiblePoint3(floor(posCheck),obstacle))
        feasible=false;dis=0;return;
    end
end
    [feasible dis]=feasiblePoint3(newPos,obstacle);
    if ~feasible, feasible=false; dis=0;
    end
end
%% feasiblePoint3.m
function [feasible dis]=feasiblePoint3(point,obstacle)
feasible=true;
% 检查点与障碍物的不同位置关系下是否碰撞
for i=1:1:length(obstacle)
     if dot(point-obstacle(i).p1,obstacle(i).p2-obstacle(i).p1)<=0     %最近点为p1
        if sqrt(sum((point-obstacle(i).p1).^2))< obstacle(i).r
            feasible = false;dis=0;return;
        else
            if i==1
            dis1=sqrt(sum((point-obstacle(i).p1).^2));
            else
            dis2=sqrt(sum((point-obstacle(i).p1).^2));
            end
        end
    elseif dot(point-obstacle(i).p1,obstacle(i).p2-obstacle(i).p1)>=dot(obstacle(i).p2-obstacle(i).p1,obstacle(i).p2-obstacle(i).p1)      %最近点为p2
        if sqrt(sum((point-obstacle(i).p2).^2))< obstacle(i).r
            feasible = false;dis=0;return;
        else
            if i==1
            dis1=sqrt(sum((point-obstacle(i).p2).^2));
            else
            dis2=sqrt(sum((point-obstacle(i).p2).^2));
            end
        end
     else  %最近点为p1p2线段上的垂足
        %计算垂足
            t=(obstacle(i).p1(1)-obstacle(i).p2(1))*(point(1)-obstacle(i).p2(1))+(obstacle(i).p1(2)-obstacle(i).p2(2))*(point(2)-obstacle(i).p2(2))+(obstacle(i).p1(3)-obstacle(i).p2(3))*(point(3)-obstacle(i).p2(3));
            t=t/sum((obstacle(i).p1-obstacle(i).p2).^2);
            xp=(obstacle(i).p1(1)-obstacle(i).p2(1))*t+obstacle(i).p2(1);
            yp=(obstacle(i).p1(2)-obstacle(i).p2(2))*t+obstacle(i).p2(2);
            zp=(obstacle(i).p1(3)-obstacle(i).p2(3))*t+obstacle(i).p2(3);
            P=[xp,yp,zp];
        if sqrt(sum((P-point).^2))< obstacle(i).r
            feasible = false;dis=0;return;
        else
            if i==1
            dis1=sqrt(sum((P-point).^2));
            else
            dis2=sqrt(sum((P-point).^2));
            end
        end
%     if sqrt(sum((P-point).^2)) < obstacle(i).r
%         if sqrt(sum((P-obstacle(i).p1).^2))>=obstacle(i).r && sqrt(sum((P-obstacle(i).p2).^2))>=obstacle(i).r
%             break;
%         else
%         feasible = false;
%         end
    end
    
end
dis=[dis1,dis2];  %返回节点与两个障碍物的最小距离
end