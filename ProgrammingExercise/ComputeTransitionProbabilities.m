function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

P = zeros(470,470,5);

for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        count_neighbours = 0;
        neighbour_type = zeros(1,4);
        if map(m, n) ~= TREE %neighbour_type has the following structure: [north south est west] where 0 represents a tree in that direction
            
            if (m > 1 && map(m-1,n) ~= TREE) %north
                count_neighbours = count_neighbours + 1;
                neighbour_type(1) = 1;
            end
            if (m < size(map, 1) && map(m+1,n) ~= TREE) %south
                count_neighbours = count_neighbours + 1;
                neighbour_type(2) = 1;
            end
            if (n < size(map,2) && map(m,n+1) ~= TREE) %est
                count_neighbours = count_neighbours + 1;
                neighbour_type(3) = 1;
            end
            if (n > 1 && map(m,n-1) ~= TREE) %west
                count_neighbours = count_neighbours + 1;
                neighbour_type(4) = 1;
            end
             
            transition_probability_value = 1/count_neighbours;
            
            for l = 1:4
                if neighbour_type(1) == 1 %north
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),l) = transition_probability_value;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,l) = transition_probability_value;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),l) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,l) = 0;
                end
                
                if neighbour_type(2) == 1 %south
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),l) = transition_probability_value;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,l) = transition_probability_value;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),l) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,l) = 0;
                end
                
                if neighbour_type(3) == 1 %est
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0),l) = transition_probability_value;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0) + 1,l) = transition_probability_value;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0),l) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0) + 1,l) = 0;
                end

                if neighbour_type(4) == 1 %west
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0),l) = transition_probability_value;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,l) = transition_probability_value;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0),l) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,l) = 0;
                end
                
            end
                 
        end
    end
end

end

