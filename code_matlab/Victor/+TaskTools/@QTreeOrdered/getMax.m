function [node,maxt] = getMax(self)
%GETMAX Summary of this function goes here
%   Detailed explanation goes here
    maxt = self.arrval(self.idx);
    node = self.array(self.idx);
end
