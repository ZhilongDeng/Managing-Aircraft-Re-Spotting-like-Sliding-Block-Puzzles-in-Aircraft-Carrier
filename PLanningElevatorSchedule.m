% Schedule one elevator movement and append the resulting row to the elevator
% schedule.
%
% Inputs:  DispatchJob - job type, aircraft number and priority in columns 6,
%          7 and 10; Level - destination level, 1 = flight deck, 2 = hangar
% Outputs: NewElevatorSchedule - only the row added by this call;
%          ElevatorInformation - spot and next available time advanced
%
% A same-level move is instantaneous; any other move costs ElevatorTransferTime.
% Columns 5, 8 and 9 of the new row (entity, tractor, trajectory priority) stay 0.
function  [NewElevatorSchedule,AllElevatorSchedule,ElevatorInformation]=PLanningElevatorSchedule(AllElevatorSchedule,DispatchJob,ElevatorID,Level,ElevatorInformation)
global ElevatorSpot ElevatorTransferTime
JobType=DispatchJob(1,6);
JobID=DispatchJob(1,7);
JobPriority=DispatchJob(1,10);
ElevatorStartSpot=ElevatorInformation(ElevatorID,2); 
ElevatorEndSpot=ElevatorSpot(Level,ElevatorID);
NewElevatorSchedule=[];
if ElevatorStartSpot~=ElevatorEndSpot
    TransferTime=ElevatorTransferTime;
else
    TransferTime=0;
end
    ElevatorStartTime=ElevatorInformation(ElevatorID,3);
    ElevatorEndTime=ElevatorStartTime+TransferTime;
    TempNewElevatorSchedule=[ElevatorStartSpot,ElevatorEndSpot,ElevatorStartTime,ElevatorEndTime,0,JobType,JobID,0,0,JobPriority];
    NewElevatorSchedule=[NewElevatorSchedule;TempNewElevatorSchedule];
    AllElevatorSchedule=[AllElevatorSchedule;TempNewElevatorSchedule];
    ElevatorInformation(ElevatorID,2:3)=[ElevatorEndSpot,ElevatorEndTime];
end
