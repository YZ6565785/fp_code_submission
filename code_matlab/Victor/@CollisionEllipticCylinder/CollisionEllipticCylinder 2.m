classdef CollisionEllipticCylinder < collisionMesh
    %COLLISIONELLIPTICCYLINDER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        function self = CollisionEllipticCylinder(x_rad,y_rad,length)
            %COLLISIONELLIPTICCYLINDER Construct an instance of this class
            %   Detailed explanation goes here
            n_points = 100;
            t = linspace(0,1,n_points+1)';
            t = t(1:end-1);
            assert(size(t,1)==n_points, "Make sure t is a "+n_points+"-by-1 matrix");

            x = sin(pi*2*t)*x_rad;
            y = cos(pi*2*t)*y_rad;
            
            z1 = zeros(size(t))+length/2;
            z2 = zeros(size(t))-length/2;

            V = [x y z1; x y z2];
            self = self@collisionMesh(V);


        end

    end
end

