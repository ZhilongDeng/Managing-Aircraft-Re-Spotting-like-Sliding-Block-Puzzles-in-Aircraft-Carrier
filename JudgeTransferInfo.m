% Choose the elevator for a cross-layer transfer: among the elevators whose
% two stage routes are both unobstructed, return the earliest available one.
%
% Stage1/Stage2 select the two stage layers (1 = flight deck, 2 = hangar) and
% index ElevatorSpot, whose row 1 holds deck hand-off spots and row 2 hangar
% ones, one column per elevator.
%
% Outputs: IsObstructive = 1 when no elevator can serve both stages, ElevatorID
% = the selected column of ElevatorSpot (0 when none).

function [IsObstructive,TrajectoryPriorityInStage1,TrajectoryPriorityInStage2,ElevatorID]=...
    JudgeTransferInfo(StartSpot,EndSpot,EntityType,SpotOccupancy,Stage1,Stage2,ElevatorSpot,ElevatorInformation)

StartSpotForStage1=StartSpot;
EndSpotForStage2=EndSpot;
ElevatorID=0;
IsObstructive=1;
TrajectoryPriorityInStage1=0;
TrajectoryPriorityInStage2=0;
ElevatorStartTime=inf;
NumOfElevator=size(ElevatorInformation,1);
for ii=1:NumOfElevator
    EndSpotForStage1=ElevatorSpot(Stage1,ii);
    StartSpotForStage2=ElevatorSpot(Stage2,ii);
    [IsObstructiveInstage1,TempTrajectoryPriorityInStage1]=JudgeDispatchInfo(StartSpotForStage1,EndSpotForStage1,EntityType,SpotOccupancy);
    [IsObstructiveInstage2,TempTrajectoryPriorityInStage2]=JudgeDispatchInfo(StartSpotForStage2,EndSpotForStage2,EntityType,SpotOccupancy);
    if ~IsObstructiveInstage1&&~IsObstructiveInstage2
        IsObstructive=0;
        % Column 3 = the elevator's earliest available time; the strict <
        % keeps the lowest-numbered elevator on ties.
        TempElevatorStartTime=ElevatorInformation(ii,3);
        if TempElevatorStartTime<ElevatorStartTime
            ElevatorID=ii;
            TrajectoryPriorityInStage1=TempTrajectoryPriorityInStage1;
            TrajectoryPriorityInStage2=TempTrajectoryPriorityInStage2;
            ElevatorStartTime=TempElevatorStartTime;
        end
    end
end
end


