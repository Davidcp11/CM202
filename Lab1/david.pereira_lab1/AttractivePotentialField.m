classdef AttractivePotentialField < handle
    % Represents an attractive potential field.
    
    properties
        kAtt; % attractive constant
        d0; % distance threshold
    end
    
    methods
        function self = AttractivePotentialField(kAtt, d0)
            % self = AttractivePotentialField(kAtt, d0) constructs an
            % attractive field potential. The inputs are the attractive
            % constant kAtt and the distance threshold d0.
            self.kAtt = kAtt;
            self.d0 = d0;
        end
        
        function U = computePotential(self, position, goal)
            % U = computePotential(self, position, goal) computes the
            % potential given the robot's position and the goal. The output
            % is the potential at the given position.
            
            % Implement
            % Distance between the robot and the goal
            dq = norm(position-goal);
            if dq <= self.d0
                U = 0.5 * self.kAtt * (dq)^2;
            else 
                U = self.d0 * self.kAtt * dq - 0.5 * self.kAtt * (self.d0)^2;
            end
        end
        
        function dU = computeGradient(self, position, goal)
            % dU = computePotential(self, position, goal) computes the
            % potential gradient given the robot's position and the goal. 
            % The output is the potential gradient at the given position.
            
            % Implement
            % Distance between the robot and the goal
            dq = norm(position-goal);
            if dq <= self.d0
                dU = self.kAtt * (position-goal);
            else 
                dU = self.d0 * self.kAtt * (position-goal) / dq;
            end
        end
    end
end