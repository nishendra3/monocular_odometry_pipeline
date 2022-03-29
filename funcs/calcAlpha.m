function [alpha] = calcAlpha(Ri, Ui , Rf, Uf, K)
% Calculating bearing vectors and deriving the angle between them
% Ri is the intial rotation, Ui is the initia vector and Rf, Uf the finals

Ui = K\[Ui';ones(1,size(Ui,1))]; % 3xM = #3x1XM
Ui = normc(Ui); % normalize columns - #3xM

prev_bearing_vector = pagemtimes(Ri,reshape(Ui, 3,1,[])); % 3x1xM
prev_bearing_vector = reshape(prev_bearing_vector, 3, []); %3xM


Uf = K\[Uf';ones(1,size(Uf,1))];
Uf = normc(Uf); %3XM
curr_bearing_vector = Rf*Uf; %3XM

alpha = atan2(vecnorm(cross(prev_bearing_vector,curr_bearing_vector)),dot(prev_bearing_vector,curr_bearing_vector));

alpha = alpha'; % M x 1
end

