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


%% Initialization


[M, N] = size(map);
P = zeros(K,K,5);
P_survive_ar = zeros(K,K,5); %Probability to survive from angry resident after being wind mooved 
P_wind = zeros(K,K,5);% Probability to be mooved from place index(a) to index(b) by the wind
%Create shooter matrix
[row_shooter, col_shooter] = find(map(:,:) == SHOOTER);
[row_base, col_base] = find(map(:,:) == BASE);
base_Ss = Ss_coor(row_base, col_base ,0, stateSpace);
[row_pickup, col_pickup] = find(map(:,:) == PICK_UP);
pickup_Ss = Ss_coor(row_pickup, col_pickup ,0 ,stateSpace);


   

%% Stage 1: Input + Out of bound and Tree feasebility check
for stateSpace_i=1:K
    for input=1:5
        if stateSpace_i == TERMINAL_STATE_INDEX
            P(stateSpace_i,stateSpace_i,input)=1;
        end
        
        for wind_d=1:5
        
        m_i = stateSpace(stateSpace_i, 1);
        n_i = stateSpace(stateSpace_i, 2);
        payload = stateSpace(stateSpace_i, 3);
        

        
        if input == NORTH
            
                if ( (n_i+1 > N) || ( map(m_i,n_i+1)==TREE ))
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input); 
                    continue
                else
                    start1 = Ss_coor(m_i,n_i+1, payload ,stateSpace);
                end
                
        elseif input == SOUTH
                if ( (n_i-1 < 1) || ( map(m_i,n_i-1)==TREE ))
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input);  
                    continue
                else
                    start1 = Ss_coor(m_i,n_i-1, payload ,stateSpace);
                end
               
        elseif input == WEST
                if ( (m_i-1 < 1) || ( map(m_i-1,n_i)==TREE ))
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input); 
                    continue
                else
                    start1 = Ss_coor(m_i-1,n_i, payload ,stateSpace);
                end
            
                
        elseif input == EAST
                if ( (m_i+1 > M) || ( map(m_i+1,n_i)==TREE ))
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input); 
                    continue
                else
                    start1 = Ss_coor(m_i+1,n_i, payload ,stateSpace);
                end
                
        elseif input == HOVER

                start1 = stateSpace_i;
        end

%% Stage 2: Wind mooved
        x1 = stateSpace(start1, 1);
        y1 = stateSpace(start1, 2);
    
        
            if wind_d == 1 %North
                if (y1+1>N) || (map(x1,y1+1)==TREE)
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start2 =Ss_coor( x1,y1 + 1, payload ,stateSpace);
                    P_wind(stateSpace_i, start2, input) = P_wind(stateSpace_i, start2, input) + P_WIND/4;
                end
                
            elseif wind_d == 2 %South
                if (y1-1 < 1) || (map(x1,y1-1)==TREE)
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start2 =Ss_coor( x1,y1 - 1, payload ,stateSpace);
                    P_wind(stateSpace_i, start2, input) = P_wind(stateSpace_i, start2, input) + P_WIND/4;
                end
            elseif wind_d == 3 %West
                if (x1-1<1) || (map(x1-1,y1)==TREE)
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start2 =Ss_coor( x1-1,y1, payload ,stateSpace);
                    P_wind(stateSpace_i, start2, input) = P_wind(stateSpace_i, start2, input) + P_WIND/4;
                end
            elseif wind_d == 4 %East
                if (x1 + 1  > M) || (map(x1+1,y1)==TREE)
                    P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start2 =Ss_coor( x1+1,y1, payload ,stateSpace);
                    P_wind(stateSpace_i, start2, input) = P_wind(stateSpace_i, start2, input) + P_WIND/4;
                end
            elseif wind_d == 5 %Stay
                start2=start1;
                
                P_wind(stateSpace_i, start2, input) = P_wind(stateSpace_i, start2, input) + 1 - P_WIND;
         end


%% Stage 3: Hit by angry residents

            x2 = stateSpace(start2, 1);
            y2 = stateSpace(start2, 2);
            dist = zeros(1, length(row_shooter));
            P_shoot = zeros(1, length(row_shooter));
            p_safe=1;
            for k=1:length(dist)
                dist(k) = abs(row_shooter(k) - x2) + abs(col_shooter(k) - y2);
                if (dist(k) <= R) && (dist(k) >= 0)
                    P_shoot(k) = GAMMA/(dist(k)+1);
                    
                else
                    P_shoot(k) = 0;
                end
            p_safe = p_safe*(1- P_shoot(k));
            end
                
                
            
            P(stateSpace_i, base_Ss, input) = P(stateSpace_i, base_Ss, input) + (1-p_safe)*P_wind(stateSpace_i, start2, input);
            P_survive_ar(stateSpace_i, start2, input) = P_survive_ar(stateSpace_i, start2, input) + p_safe*P_wind(stateSpace_i, start2, input);
            
%% Additional constraints: Pick Up , Terminal State

            if ( start2 == pickup_Ss )
                start3 = Ss_coor(x2,y2,1,stateSpace);
                P(stateSpace_i, start3, input) = P(stateSpace_i, start3, input) + P_survive_ar(stateSpace_i, start2, input);
            elseif ( start2 == TERMINAL_STATE_INDEX )
                start3 = start2 ;
                P(stateSpace_i, start3, input) = P(stateSpace_i, start3, input) + P_survive_ar(stateSpace_i, start2, input);
            else 
                start3 = start2;
                P(stateSpace_i, start3, input) = P(stateSpace_i, start3, input) + P_survive_ar(stateSpace_i, start2, input);
            end
        end
            
    end
end
end


function sS_coord=Ss_coor(s,a,pack,stateSpace)
sS_coord=find(ismember( stateSpace , [s, a, pack], 'rows'));
end