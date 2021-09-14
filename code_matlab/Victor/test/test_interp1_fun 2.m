x = 1:5
v = x.^3
xq = [1 1.5 2 2.5]
vq = interp1(x,v,xq);
figure(1);clf;hold on
plot(x,v)
plot(xq,vq,'+')
xq = linspace(1,5,100)
plot(xq,interp1(x,v,xq,'pchip'),'o')