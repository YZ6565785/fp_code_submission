%% 2D points
rng('default') % For reproducibility
X = rand(3,2);
Y = rand(4,2);
X
Y
%
[D,I] = pdist2(X,Y,'euclidean','Smallest',2)



%% 2D points
n = 2;
D = zeros(n,size(Y,1));
I = zeros(n,size(Y,1));
for j=1:size(Y,1)
    d_list = [];
    for i=1:size(X,1)
%         disp(X(i,:));
        d = sqrt((X(i,1)-Y(j,1))^2 + (X(i,2)-Y(j,2))^2 );
%         disp(d);
        d_list = [d_list d];
    end
    [M,II] = mink(d_list,n);
    D(:,j) = M;
    I(:,j) = II;
end
D
I

%% 3D points
rng('default') % For reproducibility
X = rand(3,3);
Y = rand(4,3);
X
Y
%
[D,I] = pdist2(X,Y,'euclidean','Smallest',2)

%% 3D points
n = 2;
D = zeros(n,size(Y,1));
I = zeros(n,size(Y,1));
for j=1:size(Y,1)
    d_list = [];
    for i=1:size(X,1)
%         disp(X(i,:));
        d = sqrt((X(i,1)-Y(j,1))^2 + (X(i,2)-Y(j,2))^2 + (X(i,3)-Y(j,3))^2 );
%         disp(d);
        d_list = [d_list d];
    end
    [M,II] = mink(d_list,n);
    D(:,j) = M;
    I(:,j) = II;
end
D
I