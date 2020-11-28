function [P, shooted] = ComputeTransitionProbabilities(stateSpace, map)
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

%Initializing the transition probability matrix. The probability of being
%safe after a move
P = zeros(K,K,5);

%Create shooter matrix
[row_shooter, col_shooter] = find(map(:,:) == SHOOTER);
shooter = [row_shooter'; col_shooter'];

%Create shooted matrix: probability of being shot down in a cell
shooted = ones(size(map,1), size(map,2), sum(map(:) == SHOOTER));

for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        count_neighbours = 1;
        neighbour_type = zeros(1,5);
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
            
                if neighbour_type(1) == 1 %north
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),:) = transition_probability_value * (1 - P_WIND);
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * (1 - P_WIND);
                    
                    %transition probabilities taking into account the wind
                    if m-2 > 0 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-2 & stateSpace(:,2) == n & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-2 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n-1 > 0 
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n+1 < size(map,2)
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),:) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = 0;
                end
                
                if neighbour_type(2) == 1 %south
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),:) = transition_probability_value * (1 - P_WIND);
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * (1 - P_WIND);
                    
                    %transition probabilities taking into account the wind
                    if m+2 < size(map,1)
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m+2 & stateSpace(:,2) == n & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+2 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n-1 > 0 
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n+1 < size(map,2)
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),:) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = 0;
                end
                
                if neighbour_type(3) == 1 %est
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0),:) = transition_probability_value * (1 - P_WIND);
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * (1 - P_WIND);
                    
                    %transition probabilities taking into account the wind
                    if m-1 > 0 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if m+1 < size(map,1) 
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n+2 > 0 
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+2 & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+2 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0),:) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0) + 1,:) = 0;
                end

                if neighbour_type(4) == 1 %west
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0),:) = transition_probability_value * (1 - P_WIND);
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * (1 - P_WIND);
                    
                    %transition probabilities taking into account the wind
                    if m-1 > 0 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if m+1 < size(map,1) 
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n-2 > 0 
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-2 & stateSpace(:,3) == 0),:) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-2 & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = transition_probability_value * 1/4 * P_WIND;
                else 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0),:) = 0;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,:) = 0;
                end
                
                %hover
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),5) = transition_probability_value * (1 - P_WIND);
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,5) = transition_probability_value * (1 - P_WIND);
                    
                    %transition probabilities taking into account the wind
                    if m-1 > 0 
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0),5) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m-1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,5) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n-1 > 0 
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,5) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n-1 & stateSpace(:,3) == 0) + 1,5) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if n+1 < size(map,2)
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0),5) = transition_probability_value * 1/4 * P_WIND;
                        P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n+1 & stateSpace(:,3) == 0) + 1,5) = transition_probability_value * 1/4 * P_WIND;
                    end
                    
                    if m+1 < size(map,2)
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,5) = transition_probability_value * 1/4 * P_WIND;
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m+1 & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,5) = transition_probability_value * 1/4 * P_WIND;
                    end
            
            for shooter_id = 1:sum(map(:) == SHOOTER) %iterate on the number of shooters
                distance = abs(sum(shooter(:,shooter_id) - [m;n])); %compute distance between shooter and drone
                if distance <= R %if drone is within range of the shooter
                    shooted(m,n,shooter_id) = shooted(m,n,shooter_id) + GAMMA/(distance + 1); %probability to be shot down by an angry residents
                end
            end

            
            shooted(m,n,1) = sum(squeeze(shooted(m,n,:))) - sum(triu(squeeze(permute(shooted(m,n,:), [2 1 3])) .* squeeze(shooted(m,n,:)))); %probabilities of the union of probabilies of being shot by each shooter in position (m,n)
            
            for shooter_id = 1:sum(map(:) == SHOOTER) %iterate on the number of shooters
                distance = abs(sum(shooter(:,shooter_id) - [m,n])); %compute distance between shooter and drone
                if distance <= R %if drone is within range of the shooter
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),:) = P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0),...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) ,:) * (1 - shooted(m,n,1)); %probability of being safe after a move
                    P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) = P(find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,...
                        find(stateSpace(:,1) == m & stateSpace(:,2) == n & stateSpace(:,3) == 0) + 1,:) * (1 - shooted(m,n,1)); %probability of being safe after a move
                end
            end
            
            
        end 
    end
end
end
