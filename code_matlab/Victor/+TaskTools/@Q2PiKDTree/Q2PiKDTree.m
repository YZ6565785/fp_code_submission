classdef Q2PiKDTree < TaskTools.Q2Pi
    %Q2Pi Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        l_child
        r_child
    end
    
    methods
        function self = Q2PiKDTree(q)
            %Q2Pi Construct an instance of this class
            %   Detailed explanation goes here  
            self = self@TaskTools.Q2Pi(q);
            
        end
        
        function [left_Q, right_Q] = drawTree(self)
           
           left_Q = [];
           if ~isempty(self.l_child)
               [~,left_Q] = self.findChild(self.l_child, left_Q);
               disp(left_Q);
               disp("left: "+size(left_Q,1));
           end
           right_Q = [];
           if ~isempty(self.r_child)
               [~,right_Q] = self.findChild(self.r_child, right_Q);
               disp(right_Q);
               disp("right: "+size(right_Q,1));
           end
        end
        
        function Q = getAllQ(self,Q)
           Q(1) = self;
           if ~isempty(self.l_child)
               [~,Q] = self.findChild(self.l_child, Q);
           end
           
           if ~isempty(self.r_child)
               [~,Q] = self.findChild(self.r_child, Q);
           end
           
        end
        
        function [child,save] = findChild(self, node, save)
           if isempty(node)
               child = node;
           else
              save = [save; node];
              [child,save] = self.findChild(node.l_child, save);
              if isempty(child)
                  [child,save] = self.findChild(node.r_child, save);
              end
           end
        end
    end
    
    
    
end

