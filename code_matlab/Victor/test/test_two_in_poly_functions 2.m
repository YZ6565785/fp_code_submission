

poly = polyshape();

for i=1:1000
    q = rand(1000,2);
    poly.Vertices = [1 1; 1 -1; -1 -1; -1 1];
%     figure(3);clf;hold on
%     plot(poly)
%     scatter(q(:,1), q(:,2))
    f1(q,poly.Vertices);
    f2(q,poly)
end
%%

function f1(q, poly)
    sum(inpolygon(q(:,1), q(:,2), poly(:,1), poly(:,2)));
end
function f2(q, poly)
    sum(isinterior(poly, q(:,1), q(:,2)));
end