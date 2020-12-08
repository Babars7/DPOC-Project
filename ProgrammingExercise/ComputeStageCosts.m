function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    P = ComputeTransitionProbabilities(stateSpace, map);
    INPUTS = [NORTH, SOUTH, EAST, WEST, HOVER]; 
    G = zeros(K,length(INPUTS));
    
    [m_base, n_base] = find(map == BASE);
    base_stateSpace = find(ismember(stateSpace, [m_base, n_base, 0], 'rows'));
    
    for stateSpace_i = 1:K
        m_i = stateSpace(stateSpace_i,1);
        n_i = stateSpace(stateSpace_i,2);
        
        %terminal state cost
        if (stateSpace_i == TERMINAL_STATE_INDEX)
            G(stateSpace_i,:) = 0;
            continue
        end
    
        %taking care of cases where arrival j is a tree or outside the map. inf represents an impossible input
        for input = 1:length(INPUTS)
                if input == NORTH
                    if (n_i + 1 > size(map,2) || map(m_i, n_i + 1) == TREE)
                        G(stateSpace_i,input) = inf;
                        continue
                    end
                elseif input == SOUTH
                    if (n_i - 1 < 1 || map(m_i, n_i - 1) == TREE)
                        G(stateSpace_i,input) = inf;
                        continue
                    end
                elseif input == EAST
                    if (m_i + 1 > size(map,1) || map(m_i + 1, n_i) == TREE)
                        G(stateSpace_i,input) = inf;
                        continue
                    end
                elseif input == WEST
                    if (m_i - 1 < 1 || map(m_i - 1, n_i) == TREE)
                        G(stateSpace_i,input) = inf;
                        continue
                    end   
                end
                
                for stateSpace_j = 1:K

                    if stateSpace_j == base_stateSpace
                        G(stateSpace_i,input) = G(stateSpace_i,input) + Nc * P(stateSpace_i,stateSpace_j,input);
                    else
                        G(stateSpace_i,input) = G(stateSpace_i,input) + P(stateSpace_i,stateSpace_j,input);
                    end

                end


        end
    end
end





