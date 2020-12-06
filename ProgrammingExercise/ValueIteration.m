function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER
global TERMINATION_STATE_INDEX

L = size(P,3); %number of inputs
err = 1e-2; %threshold for the termination state

    % INITIALIZE PROBLEM
    % Our state space is S=KxKxL,
    
    % Initialize costs to an arbitrary value (here 1)
    J = ones(K, 1);
    
    % Initialize the optimal control policy: 1 represents Fast serve, 2
    % represents Slow serve
    u_opt_ind = ones(K, 1);
    
    % Initialize cost-to-go
    J_opt = zeros(K, 1);
    
    % Iterate until cost has converged
    iter = 0;
    while (1)
    %for k = 1:iter
        
        % Increase counter
        iter = iter + 1
       

        for stateSpace_i = 1:K
            if stateSpace_i == TERMINATION_STATE_INDEX
                continue
            else
                [J_opt(stateSpace_i), u_opt_ind(stateSpace_i)] = min(G(stateSpace_i,:) + J'*squeeze(P(stateSpace_i,:,:)));
            end
        end
        
        if abs(J - J_opt) < err
            J = J_opt;
            iter
            break;
        else 
            J = J_opt;
        end
    end
%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)


end