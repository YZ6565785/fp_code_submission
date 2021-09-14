function T = getTaskFromProgress(self, percent) % progress: from 0 to 1
    s = size(self.task.T,3);
    ind_T = min(s, max(1, ceil(s * percent)));
    T = self.task.T(:,:,ind_T);
end