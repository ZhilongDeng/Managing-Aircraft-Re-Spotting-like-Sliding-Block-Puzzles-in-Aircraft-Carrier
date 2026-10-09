% AIRCRAFTTRANSPOSITION  Run one re-spotting request end to end: A* search for
% the aircraft move sequence, then serial scheduling of those moves.
%
% Inputs: TranspositionJob - 10-column job table; columns 1-2 hold the
%   requested start and target spots, one requested pair per row.
% Outputs: TranspositionSchedule - dispatch schedule for the found sequence,
%   [] when no sequence is found; IsCannotProceed - 1 in that case.
function [TranspositionSchedule,IsCannotProceed]=AircraftTransposition(TranspositionJob)
global SpotsOccupancy AllDeckDispatchSchedule TimeSearch TimeSchedule
% Snapshot for the search; SpotsOccupancy is advanced once the moves are
% scheduled, further down.
SpotOccupancyInThisTurn=SpotsOccupancy;
IsCannotProceed=0;
JobStartSpot=TranspositionJob(:,1);
JobEndSpot=TranspositionJob(:,2);
tic
% A* search for the move sequence; TransferCompletenessFlag is 1 only if the
% target layout is reached.
[NodeTransfer,AircraftNumber,TransferCompletenessFlag]=SpotTransposition(JobStartSpot,JobEndSpot,SpotOccupancyInThisTurn);
TimeSearch=toc
if TransferCompletenessFlag
    % One job per move; the scheduler derives each job type (column 6).
    JobList=repmat(TranspositionJob(1,:),size(NodeTransfer,1),1);
    JobList(:,1:2)=NodeTransfer(:,1:2);
    JobList(:,7)=AircraftNumber;
    tic
    [TranspositionSchedule]=AircraftTransferScheduling(JobList);
    TimeSchedule=toc
else
    IsCannotProceed=1;
    TranspositionSchedule=[];
end
end






