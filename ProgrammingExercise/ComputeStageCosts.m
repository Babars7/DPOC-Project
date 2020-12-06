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
    
    P = ComputeTransitionProbabilities_2(stateSpace, map);
    G = zeros(K,5);
    INPUTS = [NORTH, SOUTH, EAST, WEST, HOVER]; 
    
    for stateSpace_i = 1:K
        m_i = stateSpace(stateSpace_i,1);
        n_i = stateSpace(stateSpace_i,2);
        payload = stateSpace(stateSpace_i,3); %0 is no payload, 1 is payload
        
    %terminal state cost
    if (stateSpace_i == TERMINAL_STATE_INDEX)
        G(stateSpace_i,:) = 0;
        continue
    end
    
    %taking care of cases where arrival j is a tree or outside the map. inf represents an impossible input
        for l = 1:length(INPUTS)
            switch l
                case NORTH
                    if (n_i + 1 > size(map,2) || map(m_i, n_i + 1) == TREE)
                        G(stateSpace_i,l) = inf;
                        continue
                    end
                case SOUTH
                    if (n_i - 1 < 1 || map(m_i, n_i - 1) == TREE)
                        G(stateSpace_i,l) = inf;
                        continue
                    end
                case EAST
                    if (m_i + 1 > size(map,1) || map(m_i + 1, n_i) == TREE)
                        G(stateSpace_i,l) = inf;
                        continue
                    end
                case WEST
                    if (m_i - 1 < 1 || map(m_i - 1, n_i) == TREE)
                        G(stateSpace_i,l) = inf;
                        continue
                    end   
            end
        end
    
        for l = 1:length(INPUTS)
                for stateSpace_j = 1:K

                    if stateSpace_j == BASE
                        G(stateSpace_i,l) = G(stateSpace_i,l) + Nc * P(stateSpace_i,stateSpace_j,l)*G(;
                    else
                        G(stateSpace_i,l) = G(stateSpace_i,l) + P(stateSpace_i,stateSpace_j,l);
                    end

                end
        end
    

    end





