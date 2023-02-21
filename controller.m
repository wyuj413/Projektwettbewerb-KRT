function [U] = controller(X)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% function [U] = controller(X)
%
% controller for the single-track model
%
% inputs: x (x position), y (y position), v (velocity), beta
% (side slip angle), psi (yaw angle), omega (yaw rate), x_dot (longitudinal
% velocity), y_dot (lateral velocity), psi_dot (yaw rate (redundant)), 
% varphi_dot (wheel rotary frequency)
%
% external inputs (from 'racetrack.mat'): t_r_x (x coordinate of right 
% racetrack boundary), t_r_y (y coordinate of right racetrack boundary),
% t_l_x (x coordinate of left racetrack boundary), t_l_y (y coordinate of
% left racetrack boundary)
%
% outputs: delta (steering angle ), G (gear 1 ... 5), F_b (braking
% force), zeta (braking force distribution), phi (gas pedal position)
%
% files requested: racetrack.mat
%
% This file is for use within the "Project Competition" of the "Concepts of
% Automatic Control" course at the University of Stuttgart, held by F.
% Allgoewer.
%
% prepared by J. M. Montenbruck, Dec. 2013 
% mailto:jan-maximilian.montenbruck@ist.uni-stuttgart.de
%
% written by Y. Wang, Jun. 2022 
% mailto:st171307@stud.uni-stuttgart.de


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% state vector
x=X(1); % x position
y=X(2); % y position
v=X(3); % velocity (strictly positive)
beta=X(4); % side slip angle
psi=X(5); % yaw angle
omega=X(6); % yaw rate
x_dot=X(7); % longitudinal velocity
y_dot=X(8); % lateral velocity
psi_dot=X(9); % yaw rate (redundant)
varphi_dot=X(10); % wheel rotary frequency (strictly positive)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% racetrack
load('racetrack.mat','t_r'); % load right  boundary from *.mat file
load('racetrack.mat','t_l'); % load left boundary from *.mat file
t_r_x=t_r(:,1); % x coordinate of right racetrack boundary
t_r_y=t_r(:,2); % y coordinate of right racetrack boundary
t_l_x=t_l(:,1); % x coordinate of left racetrack boundary
t_l_y=t_l(:,2); % y coordinate of left racetrack boundary

% reference in sections
% load('ref.mat');
str1_y = 0:0.01:249.99;
str1_x = -1*ones(1,length(str1_y));

corn11_x = -1:-0.01:-17.5;
corn11_y = sqrt(8.25^2 - (corn11_x+9.25).^2) + 250;

corn12_x = -17.51:-0.01:-34;
corn12_y = -sqrt(8.25^2 - (corn12_x+25.75).^2)+250;

corn1_x = [corn11_x, corn12_x];
corn1_y = [corn11_y, corn12_y];

str2_y = 251.01:0.01:396.49;
str2_x = -34*ones(1,length(str2_y));

corn2_x = -34:0.01:23.99;
corn2_y = sqrt(29.^2-(corn2_x+5).^2)+396.5;

str3_y = 396.49:-0.01:313.2;
str3_x = (str3_y - 396.5).*(21-24)./(313.1924 - 396.5) + 24;

corn31_x = 21:0.01:27.5;
corn31_y = -sqrt(22.1924.^2-(corn31_x-43.1924).^2)+313.1924;

corn32_x = 27.51:0.01:34;
corn32_y = sqrt(22.1924.^2-(corn32_x-11.8076).^2)+281.8076;

corn3_x = [corn31_x,corn32_x];
corn3_y = [corn31_y,corn32_y];

str4_y = 281.80:-0.01:46.51;
str4_x = (str4_y - 281.8076).*(31-34)./(46.5-281.8076) + 34;

corn41_x = 31:0.01:45;
corn41_y = -sqrt(14.^2-(corn41_x-45).^2)+46.5;

corn42_x = 45.01:0.01:59;
corn42_y = sqrt(14.^2-(corn42_x-45).^2)+18.5;

corn43_x = 58.99:-0.01:20;
corn43_y = -sqrt(39.0288.^2-(corn43_x-20).^2)+20;

corn44_x = 19.99:-0.01:15;
corn44_y = -19.0288*ones(1,length(corn44_x));

corn45_x = 14.99:-0.01:-4.0288;
corn45_y = -sqrt(19.0288.^2-(corn45_x-15).^2);

corn4_x = [corn41_x,corn42_x,corn43_x,corn44_x,corn45_x];
corn4_y = [corn41_y,corn42_y,corn43_y,corn44_y,corn45_y];

ref_x = real([str1_x,corn1_x,str2_x,corn2_x,str3_x,corn3_x,str4_x,corn4_x]');
ref_y = real([str1_y,corn1_y,str2_y,corn2_y,str3_y,corn3_y,str4_y,corn4_y]');


clear -regexp str corn


%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE FEEDBACK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% gear config

i_g = [3.91 2.002 1.33 1 0.805]; % gear ratio
i_0 = 3.91; % motor ratio
n_max = 4800*pi/30; % maximum motor rev
S = 0; % wheel slip (neglected)
R = 0.302; % wheel radiusS
v_max = n_max*(R-R*S)./(i_g*i_0); % max. velocity at each gear

if v >= 0 && v < 0.9*v_max(1)
    G = 1;
elseif v >= 0.9*v_max(1) && v < 0.9*v_max(2)
    G = 2;
elseif v >= 0.9*v_max(2) && v < 0.9*v_max(3)
    G = 3;
elseif v >= 0.9*v_max(3) && v < 0.9*v_max(4)
    G = 4;
elseif v >= 0.9*v_max(4)
    G = 5;
else
    G = 1;
end

% search the reference in a neighborhood of current position

cur_ref_x = ref_x(find(x-15 <= ref_x & ref_x <= x+15 & y-15 <= ref_y & ref_y <= y+15));
cur_ref_y = ref_y(find(x-15 <= ref_x & ref_x <= x+15 & y-15 <= ref_y & ref_y <= y+15));

cur_ref = [cur_ref_x,cur_ref_y];

% Trap
if isempty(cur_ref) || size(cur_ref,1) == 1
    error('Car goes too wide!');
end

% steering angle config

sz = size(cur_ref,1); % size of point

% vector from current position to the reference trajectory
d = [x,y] - cur_ref;
d_x = d(:,1);
d_y = d(:,2);

% find the nearest point on reference line
[~, idx] = min(d_x.^2 + d_y.^2);

% Local coordinates aligning tangent and normal line of the reference
if idx+1 <= sz
    ix_ref = cur_ref(idx+1,:)-cur_ref(idx,:);   % vector aligning tangent line
elseif idx-1 >= 1
    ix_ref = cur_ref(idx,:)-cur_ref(idx-1,:);   % vector aligning tangent line
else
    error('Car goes too wide!');
end

psi_ref = atan2(ix_ref(2),ix_ref(1));       % angle according to ground coordinates
iy_ref = ix_ref*[0,-1;1 0]';                % vector aligning normal line (rotate +90 deg)

% reference error
ed = dot(-iy_ref, d(idx,:))./norm(iy_ref);                 % projection of d on -iy_ref
ed_dot = dot(-iy_ref, [x_dot,y_dot])./norm(iy_ref);        % projection of v on -iy_ref


% adjust angle s.t. it lays in [-pi,pi]
if psi_ref < 0
    psi_ref = psi_ref + 2*pi;
end

dpsi = psi_ref - psi;

if abs(dpsi) > pi
    if dpsi > 0
        dpsi = dpsi - 2*pi;
    elseif dpsi < 0
        dpsi = dpsi + 2*pi;
    end
end

% control input
kp_del = 10; % should be positive
kd_del = 0.3;
kp_psi = 50;
delta = kp_del * ed + kd_del * ed_dot + kp_psi * dpsi; % steering angle


% Brake and throttle config
Fbmax = 1.5e4;
if y >= 0 && y <= 200 && x >= -10 && x <= 5                % straight 1
    Fb = 0*Fbmax;
    phi = 1;

elseif (y > 200 && y <= 250 && x >= -40 && x <= 5) || ...
       (y > 250 && y <= 265 && x >= -25 && x <= 5) % corner 1
    v_ref = 6;
    if v >= v_ref
        Fb = 1*Fbmax;
        phi = 0;
    else
        Fb = 0;
        kpv = 10;
        phi = -kpv*(v-v_ref)*(v<v_ref);
    end
    
elseif y >= 265 && y <= 360 && x >= -40 && x < -25    % straight 2
    Fb = 0*Fbmax;
    phi = 1;

elseif (y > 360 && y <= 440 && x >= -40 && x <= -5) || ...
    (y > 400 && y <= 440 && x > -5 && x <= 30) % corner 2
    v_ref = 12;
    if v >= v_ref
    Fb = 1*Fbmax;
    phi = 0;
    else
    Fb = 0;
    kpv = 10;
    phi = -kpv*(v-v_ref)*(v<v_ref);
    end
    
elseif y > 330 && y <= 400 && x >= 18 && x <= 27       % straight 3
    Fb = 0;
    phi = 1;

elseif y > 280 && y <= 330 && x >= 18 && x <= 37 % corner 3
    
    v_ref = 10;
    if v >= v_ref
        Fb = 1*Fbmax;
        phi = 0;
    else
        Fb = 0;
        kpv = 1;
        phi = -kpv*(v-v_ref)*(v<v_ref);
    end

elseif y > 80 && y <= 280 && x >= 27.5 && x <= 40        % straight 4
    Fb = 0;
    phi = 1;

elseif y > 20 && y <= 80 && x >= 25 && x <= 65           % corner 4-1
    v_ref = 8;
    if v >= v_ref
        Fb = 1*Fbmax;
        phi = 0;
    else
        Fb = 0;
        kpv = 10;
        phi = -kpv*(v-v_ref)*(v<v_ref);
    end
    
elseif y > -25 && y <= 20 && x >= 20 && x <= 65          % corner 4-2
    v_ref = 14;
    if v >= v_ref
        Fb = 1*Fbmax;
        phi = 0;
    else
        Fb = 0;
        kpv = 10;
        phi = -kpv*(v-v_ref)*(v<v_ref);
    end
elseif y > -25 && y <= 0 && x >= -10 && x < 20          % corner 4-3
    v_ref = 8;
    if v >= v_ref
        Fb = 1*Fbmax;
        phi = 0;
    else
        Fb = 0;
        kpv = 10;
        phi = -kpv*(v-v_ref)*(v<v_ref);
    end

else
    Fb = 0;
    phi = 0;
end

zeta=0.5; % braking force distribution

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
U=[delta G Fb zeta phi]; % input vector
end

