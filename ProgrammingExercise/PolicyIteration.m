function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)




%% initialization

    err = 1e-3; %threshold for the termination state

    % INITIALIZE PROBLEM
    % Our state space is S=KxKxL,
    
    % Initialize costs to an arbitrary value (here 1)
    J = zeros(K, 1);
    
    % Initialize the optimal control policy: 1 represents Fast serve, 2
    % represents Slow serve
    u_opt_ind = HOVER.*ones(K, 1);
    u_opt_ind(TERMINAL_STATE_INDEX) = HOVER;
    
    % Initialize cost-to-go
    J_opt = ones(K, 1);
    J_opt(TERMINAL_STATE_INDEX) = 0;
    
    % Iterate until cost has converged
    iter = 0;
    iter_max = 1e4;
    
    while (1)

        
        % Increase counter
        iter = iter + 1;

        for stateSpace_i = 1:K
            if stateSpace_i == TERMINAL_STATE_INDEX
                continue
            else
                J_opt(stateSpace_i) = G(stateSpace_i,u_opt_ind(stateSpace_i)) + J_opt' * P(stateSpace_i, :, u_opt_ind(stateSpace_i))';
                
                [~, u_opt_ind(stateSpace_i)] = min(G(stateSpace_i,:) + J_opt'*squeeze(P(stateSpace_i, :, :)));
                
                
            end
        end
        
        if (iter == iter_max)
            break;
        elseif (abs(J - J_opt) < err)
            J = J_opt;
            break;
        else
            
            J = J_opt;
            
        end
        
    end

end
