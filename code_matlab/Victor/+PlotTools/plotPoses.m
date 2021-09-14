function q = plotPoses(poses,scale)
%PLOT_POSES Summary of this function goes here
%   Detailed explanation goes here


    r=quatrotate(quatinv(poses(:,4:end)), [1 0 0]);
    poses(:,1:3)=round(poses(:,1:3),8);
    r=round(r,8);
    q=quiver3(poses(:,1),poses(:,2),poses(:,3),r(:,1),r(:,2),r(:,3),scale,'r');
    hold on
    r=quatrotate(quatinv(poses(:,4:end)),[0 1 0]);
    poses(:,1:3)=round(poses(:,1:3),8);
    r=round(r,8);
    q=quiver3(poses(:,1),poses(:,2),poses(:,3),r(:,1),r(:,2),r(:,3),scale,'g');
    hold on
    r=quatrotate(quatinv(poses(:,4:end)),[0 0 1]);
    poses(:,1:3)=round(poses(:,1:3),8);
    r=round(r,8);
    q=quiver3(poses(:,1),poses(:,2),poses(:,3),r(:,1),r(:,2),r(:,3),scale,'b');
    
    axis equal
    xlabel('x')
    ylabel('y')
    zlabel('z')
end
