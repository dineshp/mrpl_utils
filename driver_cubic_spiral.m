%% CUBICSPIRAL
clc;
close all;
%CubicSpiralTrajectory.graphSpirals(1, 10000);
h = figure(1);
h.Units = 'normalized';
h.OuterPosition = [0 0 1 1];
figure(h);
CubicSpiralTrajectory.graphSpirals(1000, h);

%%
CubicSpiralTrajectory.makeLookupTable(1.8);