function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

global DROP_OFF
global K

[m_dropoff,n_dropoff] = find(map == DROP_OFF);

 for stateSpace_i = 1:K
        m_i = stateSpace(stateSpace_i,1);
        n_i = stateSpace(stateSpace_i,2);
        payload = stateSpace(stateSpace_i,3); %0 is no payload, 1 is payload
        
        if (m_i == m_dropoff && n_i == n_dropoff && payload == 1)
            stateIndex = stateSpace_i;
        end
 end
                  
end
