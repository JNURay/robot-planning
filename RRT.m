% 实现了胶囊体的碰撞检测计算，并能够输出采样点到障碍物的距离
%% 绘制障碍物(圆柱体)
ob1.p1=[50,50,50];
ob1.p2=[50,50,100];
ob1.r=20;%半径
ob2.p1=[80,100,50];
ob2.p2=[80,100,150];
ob2.r=10;
obstacle=[ob1,ob2];
%下面开始画
figure(1);
cylinder3(ob1.p1,ob1.p2,ob1.r,50,'b',1,0);
cylinder3(ob2.p1,ob2.p2,ob2.r,50,'b',1,0);
axis equal
%% 参数
source=[10 10 10];
goal=[150 150 150];
stepsize = 10; %步长
threshold = 10; %到达目标点周围的阈值
maxFailedAttempts = 10000;  %探索次数
display = true; 
searchSize = [250 250 250];      %探索空间六面体
%% 绘制起点和终点
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
tic;  % tic-toc: Functions for Elapsed Time
[feasible dis]=feasiblePoint3(source, obstacle)
RRTree = double([source -1 dis]);
failedAttempts = 0;
pathFound = false;

%% 循环
while failedAttempts <= maxFailedAttempts  % loop to grow RRTs
    %% chooses a random configuration
    if rand < 0.5
        sample = rand(1,3) .* searchSize;   % random sample
    else
        sample = goal; % sample taken as goal to bias tree generation to goal
    end
    %% selects the node in the RRT tree that is closest to qrand
    [A, I] = min( distanceCost(RRTree(:,1:3),sample) ,[],1); % find the minimum value of each column
    closestNode = RRTree(I(1),1:3);
    %% moving from qnearest an incremental distance in the direction of qrand
    movingVec = [sample(1)-closestNode(1),sample(2)-closestNode(2),sample(3)-closestNode(3)];
    movingVec = movingVec/sqrt(sum(movingVec.^2));  %单位化
    newPoint = closestNode + stepsize * movingVec;
    [feasible dis]=checkPath3(closestNode, newPoint, obstacle);
    if ~feasible % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1;
        continue;
    end
    
    if distanceCost(newPoint,goal) < threshold, pathFound = true; break; end % goal reached
    [A, I2] = min( distanceCost(RRTree(:,1:3),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree(I2(1),1:3)) < threshold, failedAttempts = failedAttempts + 1; continue; end 
    
    RRTree = [RRTree; newPoint I(1) dis]; % add node
    failedAttempts = 0;
    if display, plot3([closestNode(1);newPoint(1)],[closestNode(2);newPoint(2)],[closestNode(3);newPoint(3)],'LineWidth',1); end
    pause(0.05);
end

if display && pathFound, plot3([closestNode(1);goal(1)],[closestNode(2);goal(2)],[closestNode(3);goal(3)]); end

if display, disp('click/press any key'); waitforbuttonpress; end
if ~pathFound, error('no path found. maximum attempts reached'); end
%% retrieve path from parent information
[feasible dis]=feasiblePoint3(goal,obstacle)
path = [goal I(1) dis];
prev = I(1);
while prev > 0
    path = [RRTree(prev,1:6); path];
    prev = RRTree(prev,4);
end

pathLength = 0;
for i=1:length(path(:,1))-1, pathLength = pathLength + distanceCost(path(i,1:3),path(i+1,1:3)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \n\n', toc, pathLength); 
figure(2)
cylinder3(ob1.p1,ob1.p2,ob1.r,50,'b',1,0);
cylinder3(ob2.p1,ob2.p2,ob2.r,50,'b',1,0);
axis equal
hold on;
scatter3(source(1),source(2),source(3),"filled","g");
scatter3(goal(1),goal(2),goal(3),"filled","b");
plot3(path(:,1),path(:,2),path(:,3),'LineWidth',2,'color','r');
%显示路径点到障碍物的距离
figure(3)
plot(path(:,5:6));
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
% check if collission-free spot and inside maps
for i=1:1:length(obstacle)
t=(obstacle(i).p1(1)-obstacle(i).p2(1))*(point(1)-obstacle(i).p2(1))+(obstacle(i).p1(2)-obstacle(i).p2(2))*(point(2)-obstacle(i).p2(2))+(obstacle(i).p1(3)-obstacle(i).p2(3))*(point(3)-obstacle(i).p2(3));
t=t/sum((obstacle(i).p1-obstacle(i).p2).^2);
xp=(obstacle(i).p1(1)-obstacle(i).p2(1))*t+obstacle(i).p2(1);
yp=(obstacle(i).p1(2)-obstacle(i).p2(2))*t+obstacle(i).p2(2);
zp=(obstacle(i).p1(3)-obstacle(i).p2(3))*t+obstacle(i).p2(3);
P=[xp,yp,zp];
     if dot(point-obstacle(i).p1,obstacle(i).p2-obstacle(i).p1)<=0
        if sqrt(sum((point-obstacle(i).p1).^2))< obstacle(i).r
            feasible = false;dis=0;return;
        else
            if i==1
            dis1=sqrt(sum((point-obstacle(i).p1).^2));
            else
            dis2=sqrt(sum((point-obstacle(i).p1).^2));
            end
        end
    elseif dot(point-obstacle(i).p1,obstacle(i).p2-obstacle(i).p1)>=dot(obstacle(i).p2-obstacle(i).p1,obstacle(i).p2-obstacle(i).p1)
        if sqrt(sum((point-obstacle(i).p2).^2))< obstacle(i).r
            feasible = false;dis=0;return;
        else
            if i==1
            dis1=sqrt(sum((point-obstacle(i).p2).^2));
            else
            dis2=sqrt(sum((point-obstacle(i).p2).^2));
            end
        end
    else
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
dis=[dis1,dis2];
end