close all; clc; clear;
%% %%%%%%%%%%%% Scurve MultiAxis with Jerk Limitation
% Date: 2021/01/22
% =================================================================================
%% Limitation

% >>>> ITRI_parameter
[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

%% Scurve Single Axis Demo
% >>>> Initial

% Scurve = Scurve_Single_Axis();
% 
% for i = 1:8
%     Scurve.Demo(i);
%     figure('name', sprintf('Figure %d' , i));
%     Scurve.Plot_demo;
%     Scurve.Clear;
% end
% Scurve.Clear;

%% Scurve Multi Axis
InitialPosition = [0 10   0 -20 30 10];
FinalPosition   = [50 1 -30  0 10  5];
Scurve = Scurve_Jerk();
Scurve.Input(   InitialPosition,            FinalPosition, ...
                ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
                ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 3, ...
                sqrt(3)/2, SamplingTime);
            
Scurve.MultiAxis_Cmd;
Scurve.MultiAxis_plot;
