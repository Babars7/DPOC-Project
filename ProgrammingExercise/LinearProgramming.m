function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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
global NORTH SOUTH EAST WEST HOVER
global TERMINAL_STATE_INDEX

INPUTS = [NORTH, SOUTH, EAST, WEST, HOVER]; 

%We will solve the Linear Programming Theorem: minimize f'*x, subject to a constraint M*x
%<= h. Let's compute M and h.
h = []; 
M = [];

for i = 1:K
    h = [h, G(i,:)];
end

h(h==Inf) = 1e10;

for i = 1:K
    temp = zeros(5, K);
    temp(:, i) = 1;
    temp = temp - squeeze(P(i, :, :))';
    M =  [M; temp];
end

f = -1 * ones(1,K);

%delete the terminal state index to avoid getting an unbounded problem
f(TERMINAL_STATE_INDEX) = [];
M(:,TERMINAL_STATE_INDEX) =[];


[J_opt,~,~,~,lambda] = linprog(f,M,h);
u_opt_ind = mod(find(lambda.ineqlin ~= 0),5);
u_opt_ind(u_opt_ind == 0) = 5;

%replace the terminal state index in u_opt_ind and set it to HOVER
u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX-1); HOVER; u_opt_ind(TERMINAL_STATE_INDEX:end)];

%replace the terminal state index in J_opt and set it it to 0
J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1); 0; J_opt(TERMINAL_STATE_INDEX:end)];

end

