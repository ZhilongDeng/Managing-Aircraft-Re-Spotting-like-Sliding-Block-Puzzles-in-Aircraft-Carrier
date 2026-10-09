% JudgeDispatchInfo
% Single-layer feasibility test for a spot-to-spot transfer: decide whether any
% obstruction-free trajectory variant exists and which one is fastest. Called
% directly for single-layer jobs; cross-layer jobs reach it through
% JudgeTransferInfo, once per stage per elevator.
% Inputs: StartSpot/EndSpot = row/column index of the trajectory libraries;
%   EntityType 1 = traction system, 2 = aircraft taxiing, 3 = tractor only;
%   SpotOccupancy = occupied spots.
% Outputs: IsObstructive 0 = feasible, else 1; TrajectoryPriority = chosen
%   variant index (3rd dimension), 1 for a zero-length move, 0 if none usable.
function [IsObstructive,TrajectoryPriority]=JudgeDispatchInfo(StartSpot,EndSpot,EntityType,SpotOccupancy)
global DTTS DTAT DTT
global OLTS OLAT OLT
if EntityType==1; NumOfTrajectory=size(OLTS,3);
elseif EntityType==2; NumOfTrajectory=size(OLAT,3);
end
IsObstructive=1;
TrajectoryPriority=0;
DispatchTime=inf;
if StartSpot==EndSpot
    IsObstructive=0;
    TrajectoryPriority=1;
end
for ii=1:NumOfTrajectory
    if EntityType==1; ObstructiveList=OLTS{StartSpot,EndSpot,ii};TempDisPatchTime=DTTS{StartSpot,EndSpot,ii};
    elseif EntityType==2; ObstructiveList=OLAT{StartSpot,EndSpot,ii};TempDisPatchTime=DTAT{StartSpot,EndSpot,ii};
    elseif EntityType==3; ObstructiveList=OLT{StartSpot,EndSpot,ii};TempDisPatchTime=DTT{StartSpot,EndSpot,ii};
    end
    ObstructiveObstacle=intersect(SpotOccupancy,ObstructiveList);
    % Usable only if the obstacle list is non-empty (empty = no stored
    % trajectory) and it shares no spot with SpotOccupancy; the strict <
    % keeps the lowest variant index on ties.
    if ~isempty(ObstructiveList)&&isempty(ObstructiveObstacle)
        IsObstructive=0;
        if TempDisPatchTime<DispatchTime
            TrajectoryPriority=ii;
            DispatchTime=TempDisPatchTime;
        end
    end
end
end
