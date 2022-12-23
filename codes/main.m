%% Information
%%% Project: Controlling Multiple Robots
%%% Programmer: Mostafa Lotfi
%%% Date: 6/6/2022
%%% Matlab version: 2021a

%%% Brief Description: Controlling multiple holonomic mobile robot has been
%%% considered in this project. The task is to transport robots from their
%%% initial conditions to their desired destinations without colliding to
%%% each other. The solution is based on potential field of each robot.
%%% Hence no trajectory planning is done for robots to perform the
%%% scenario. In this approach, in every time steps, distances and
%%% directions of robots to each other and the walls will be calculated.
%%% So robots try to go to their target positions in such a way that
%%% haven't any collision with neither other robots nor the walls.
clc;
clear;
close all;

%% parameters
% plotting
static_plot = false;
dynamic_plot = true;

%% Simulation
[vars, params] = simulate_system();

%% Plotting
plot_results(vars, params, static_plot, dynamic_plot)
