% x = linspace(0.1, 0., 10)';
% n_x = size(x,1);
% points = [x zeros(n_x,1) zeros(n_x,1) ];
wave_length = 1

pipe_scale = 1;
angle = pi;
x_rad = wave_length/4;
y_rad = 0.2;

t = linspace(0, 1, 100)';

points = [cos(angle * t)*x_rad+x_rad zeros(size(t)) sin(angle * t)*y_rad ];

points = [points; cos(angle * t)*x_rad-x_rad zeros(size(t)) -sin(angle * t)*y_rad ];

figure(1);clf;hold on;axis equal
scatter3(points(:,1),points(:,2),points(:,3))

%%
S = TaskTools.SPath(1, 0.4);
points = S.path;

% plot
figure(1);clf;hold on;axis equal
scatter3(points(:,1),points(:,2),points(:,3),'filled');



%%
wave = TaskTools.WavePath()
points = wave.path

% plot
figure(1);clf;hold on;axis equal
scatter3(points(:,1),points(:,2),points(:,3));
%%

S = TaskTools.SPath(0.5, 0.3)
points = S.path;

% plot
figure(1);clf;hold on;axis equal
scatter3(points(:,1),points(:,2),points(:,3),'filled');

n_S = size(S.path,1);
s_len = S.x_rad*4

cl = wave.cumlen(); % cumulative length
num = floor(cl(end)/s_len)
num_mod = mod(cl(end),s_len)

density = 0.01;
path = zeros(num*1/density + (num_mod/density), 3)
size(path)

x = linspace(0,1,n_S)';

for i=1:ceil(cl(end)/s_len)
    i0 = (i-1)*1/density+1;
    if i<=num
        iend = i*1/density; 
        xq = linspace(0,1,1/density)';
    else
        iend = i0+num_mod/density-1;  
        xq = linspace(0,num_mod,num_mod/density)';
    end

    interp_fun = @(a) interp1(x, S.path(:,a), xq,'pchip'); % pchip: cubic approx
    xs = interp_fun(1);
    ys = interp_fun(2);
    zs = interp_fun(3);
    
    path(i0:iend,:) = [xs+s_len*(i-1) ys zs];
end

points = path;

% plot
figure(1);clf;hold on;axis equal
scatter3(points(:,1),points(:,2),points(:,3));
%%
wave = TaskTools.WavePath()
points = wave.path
vec = TForm.tform2vec(wave.T);
% plot
figure(1);clf;hold on;axis equal
scatter3(points(:,1),points(:,2),points(:,3));

figure(2);clf;hold on;axis equal
PlotTools.plotPoses(vec,0.1);

for i=1:size(wave.path,1)
   scatter3(wave.path(i,1), wave.path(i,2), wave.path(i,3),'r') 
   drawnow
end

% vec = TForm.tform2vec(wave.pointsToTForm());
% figure(3);clf;hold on;axis equal
% PlotTools.plotPoses(vec,0.1);
% 
% for i=1:size(wave.points,1)
%    scatter3(wave.points(i,1), wave.points(i,2), wave.points(i,3)) 
%    drawnow
% end
%%
inds = mod(cl,s_len)<=0.01;
path = zeros( (size(S.path,1)-1)*sum(inds), 3 );

for i=1:sum(inds)-1
   path(i:i+size(S.path,1)-1,:) = S.path(1:end-1,:)
end
% ind = (1:size(wave.path,1))';
% ind_mod = ind(mod(cl, 0.1)<=0.01);
% 
% % if ind_mod(end,1)~=1
% %     ind_mod(end,1)
% % end
% for i=1:size(ind_mod,1)
%     if i~=size(ind_mod,1)
%         ind_0 = ind_mod(i);
%         ind_1 = ind_mod(i+1);
%         
%         wave.path(ind_0:ind_1,:) = 
%     end
% end

%%
q = Tasks.UPath(0.01*2);
q.scale([0.2 0.05 1]);
points = q.path;

% plot
figure(1);clf;hold on;axis equal
scatter3(points(:,1),points(:,2),points(:,3),'filled');