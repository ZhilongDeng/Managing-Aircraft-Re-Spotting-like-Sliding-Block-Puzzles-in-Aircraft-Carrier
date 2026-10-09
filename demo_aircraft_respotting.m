% Entry-point script for the re-spotting pipeline: it sets the occupied-spot
% list and one hard-coded transfer request (the aircraft on spot 137 to the
% free spot 140, moved by the traction system), loads the configuration with
% InitTransferParameters, runs the top-level solver, and plots the resulting
% dispatch Gantt chart.
%
% Outputs:
%   NewDispatchSchedule - dispatch schedule for the request.
%   IsCannotProceed - 1 if the request could not be solved, else 0.
close all
clear all
clc

%% Load configuration and spot layout
global TLTS TLAT TLT % trajectory library (TS traction system, AT aircraft taxiing, T tractor)
global DTTS DTAT DTT % dispatch time table (same three variants)
global OLTS OLAT OLT % obstruction list (same three variants)
global AllDeckDispatchSchedule AllHangerDispatchSchedule AllElevatorSchedule
global ElevatorInfo DeckTractorInfo HangerTractorInfo
global SpotInfo TranspositionMap
global SpotsOccupancy
global TimeInitialize TimeSearch TimeSchedule
tic
InitialSpotsOccupancy=[137 138 143 144 149 152 158 159 160 161 162 163 165 166 167 169 171 172 173 175 176];
% TranspositionJob is a 10-column row: 1 start spot, 2 end spot, 3 start
% time, 4 end time, 5 entity type (1 traction system, 2 aircraft taxiing,
% 3 tractor only), 6 job type, 7 aircraft number, 8 tractor number,
% 9 trajectory priority, 10 job priority. The job type (column 6) is
% derived by the solver from the start and end spot classes.
TranspositionJob=[137 140 0 0 1 0  1 0 0 0];
SpotsOccupancy=InitialSpotsOccupancy;
InitTransferParameters;
TimeInitialize=toc
%% Call the top-level solver
[NewDispatchSchedule,IsCannotProceed]=AircraftTransposition(TranspositionJob);
plotGant(NewDispatchSchedule,InitialSpotsOccupancy)
