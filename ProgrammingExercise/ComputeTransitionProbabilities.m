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


%% A CHECKER
%-l'orientation de NORTH;SOUTH .....


%% Initialization


[M, N] = size(map);
P = zeros(K,K,5);
P_survive_ar = zeros(K,K,5); %Probability to survive from angry resident after being wind mooved 
P_w = zeros(K,K,5);% Probability to be mooved from place index(a) to index(b) by the wind
%Create shooter matrix
[row_shooter, col_shooter] = find(map(:,:) == SHOOTER);
[row_base, col_base] = find(map(:,:) == BASE);
base_Ss = Ss_coor(row_base, col_base ,0, stateSpace);
[row_pickup, col_pickup] = find(map(:,:) == PICK_UP);
pickup_Ss = Ss_coor(row_pickup, col_pickup ,0 ,stateSpace);


   

%% Stage 1: Input + Out of bound and Tree feasebility check
for start_stag0=1:K
    for input=1:5
        if start_stag0 == TERMINAL_STATE_INDEX
            P(start_stag0,start_stag0,input)=1;
        end
        
        for wind_d=1:5
        
        x_stag0 = stateSpace(start_stag0, 1);
        y_stag0 = stateSpace(start_stag0, 2);
        p = stateSpace(start_stag0, 3);
        

        
        if input == NORTH
            
                if ( (y_stag0+1 > N) || ( map(x_stag0,y_stag0+1)==TREE ))
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input); 
                    continue
                else
                    start_stag1 = Ss_coor(x_stag0,y_stag0+1, p ,stateSpace);
                end
                
        elseif input == SOUTH
                if ( (y_stag0-1 < 1) || ( map(x_stag0,y_stag0-1)==TREE ))
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input);  
                    continue
                else
                    start_stag1 = Ss_coor(x_stag0,y_stag0-1, p ,stateSpace);
                end
               
        elseif input == WEST
                if ( (x_stag0-1 < 1) || ( map(x_stag0-1,y_stag0)==TREE ))
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input); 
                    continue
                else
                    start_stag1 = Ss_coor(x_stag0-1,y_stag0, p ,stateSpace);
                end
            
                
        elseif input == EAST
                if ( (x_stag0+1 > M) || ( map(x_stag0+1,y_stag0)==TREE ))
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input); 
                    continue
                else
                    start_stag1 = Ss_coor(x_stag0+1,y_stag0, p ,stateSpace);
                end
                
        elseif input == HOVER

                start_stag1 = start_stag0;
        end

%% Stage 2: Wind mooved
        x_stag1 = stateSpace(start_stag1, 1);
        y_stag1 = stateSpace(start_stag1, 2);
    
        
            if wind_d == 1 %North
                if (y_stag1+1>N) || (map(x_stag1,y_stag1+1)==TREE)
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start_stag2 =Ss_coor( x_stag1,y_stag1 + 1, p ,stateSpace);
                    P_w(start_stag0, start_stag2, input) = P_w(start_stag0, start_stag2, input) + P_WIND/4;
                end
                
            elseif wind_d == 2 %South
                if (y_stag1-1 < 1) || (map(x_stag1,y_stag1-1)==TREE)
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start_stag2 =Ss_coor( x_stag1,y_stag1 - 1, p ,stateSpace);
                    P_w(start_stag0, start_stag2, input) = P_w(start_stag0, start_stag2, input) + P_WIND/4;
                end
            elseif wind_d == 3 %West
                if (x_stag1-1<1) || (map(x_stag1-1,y_stag1)==TREE)
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start_stag2 =Ss_coor( x_stag1-1,y_stag1, p ,stateSpace);
                    P_w(start_stag0, start_stag2, input) = P_w(start_stag0, start_stag2, input) + P_WIND/4;
                end
            elseif wind_d == 4 %East
                if (x_stag1 + 1  > M) || (map(x_stag1+1,y_stag1)==TREE)
                    P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input) + P_WIND/4;
                    continue
                else
                    start_stag2 =Ss_coor( x_stag1+1,y_stag1, p ,stateSpace);
                    P_w(start_stag0, start_stag2, input) = P_w(start_stag0, start_stag2, input) + P_WIND/4;
                end
            elseif wind_d == 5 %Stay
                start_stag2=start_stag1;
                
                P_w(start_stag0, start_stag2, input) = P_w(start_stag0, start_stag2, input) + 1 - P_WIND;
         end


%% Stage 3: Hit by angry residents

            x_stag2 = stateSpace(start_stag2, 1);
            y_stag2 = stateSpace(start_stag2, 2);
            dist = zeros(1, length(row_shooter));
            P_shoot = zeros(1, length(row_shooter));
            p_safe=1;
            for k=1:length(dist)
                dist(k) = abs(row_shooter(k) - x_stag2) + abs(col_shooter(k) - y_stag2);
                if ((dist(k) <= R) && (dist(k) >= 0))
                    P_shoot(k) = GAMMA/(dist(k)+1);
                    
                else
                    P_shoot(k) = 0;
                end
            p_safe = p_safe*(1- P_shoot(k));
            end
                
                
            
            P(start_stag0, base_Ss, input) = P(start_stag0, base_Ss, input) + (1-p_safe)*P_w(start_stag0, start_stag2, input);
            P_survive_ar(start_stag0, start_stag2, input) = P_survive_ar(start_stag0, start_stag2, input) + p_safe*P_w(start_stag0, start_stag2, input);
            
%% Additional constraints: Pick Up , Terminal State

            if ( start_stag2 == pickup_Ss )
                start_stag3 = Ss_coor(x_stag2,y_stag2,1,stateSpace);
                P(start_stag0, start_stag3, input) = P(start_stag0, start_stag3, input) + P_survive_ar(start_stag0, start_stag2, input);
            elseif ( start_stag2 == TERMINAL_STATE_INDEX )
                start_stag3 = start_stag2 ;
                P(start_stag0, start_stag3, input) = P(start_stag0, start_stag3, input) + P_survive_ar(start_stag0, start_stag2, input);
            else 
                start_stag3 = start_stag2;
                P(start_stag0, start_stag3, input) = P(start_stag0, start_stag3, input) + P_survive_ar(start_stag0, start_stag2, input);
            end
        end
            
        end
    end
end

            
            
            




function sS_coord=Ss_coor(s,a,pack,stateSpace)
sS_coord=find(ismember( stateSpace , [s, a, pack], 'rows'));
end

