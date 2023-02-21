%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% init
% Initialization of the program
%
% functions called: simulate_controlled_singletrack(t_f) 
%
% This file is for use within the "Project Competition" of the "Concepts of
% Automatic Control" course at the University of Stuttgart, held by F.
% Allgoewer.
%
% written by Y. Wang, Jun. 2022 
% mailto:st171307@stud.uni-stuttgart.de

t_f = 79;   % simulation time for the program

% Start the simulation and records the elapsed simulation time in real life
tic
simulate_controlled_singletrack(t_f);
toc
