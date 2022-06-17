%% Controlling some mobile robots to go points A to B without collision
% Programmer: Mostafa Lotfi
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

