% PLanningDispatchSchedule
% Schedule one same-layer transfer job: if the job needs a tractor, choose one
% and append its pre-positioning move first, then route the aircraft. Rows are
% appended to AllDispatchSchedule and also returned in NewDispatchSchedule.
% Job rows have 10 columns: 1 start spot, 2 end spot, 3 start time, 4 end time,
% 5 entity type (1 traction system, 2 aircraft taxiing, 3 tractor only), 6 job
% type, 7 aircraft number, 8 tractor number, 9 trajectory priority, 10 job priority.
% TractorInformation: one row per tractor; col 2 current spot, col 3 earliest
% time the tractor is free.
function [NewDispatchSchedule,AllDispatchSchedule,TractorInformation]=PLanningDispatchSchedule(AllDispatchSchedule,DispatchJob,TractorInformation)

global DTTS DTAT DTT % dispatch time tables: traction system / aircraft taxiing / tractor
% Job end time (col 4) and tractor (col 8) are not read: this function decides both.
StartSpot=DispatchJob(1,1);
EndSpot=DispatchJob(1,2);
StartTime=DispatchJob(1,3);
EntityType=DispatchJob(1,5);
JobType=DispatchJob(1,6);
JobID=DispatchJob(1,7);
TrajectoryPriority=DispatchJob(1,9);
JobPriority=DispatchJob(1,10);
TractorID=0; % no tractor chosen yet
NewDispatchSchedule=[];
NumOfTractor=size(TractorInformation,1);
% Only EntityType 1 jobs need a tractor; they cannot start before it arrives.
if EntityType==1
    % TempDispathForAllDeckTractor cols: 1 tractor index, 2 earliest arrival at
    % StartSpot, 3 travel time from the tractor's current spot.
    TempDispathForAllDeckTractor=zeros(NumOfTractor,3);
    for ii=1:NumOfTractor
        TractorStartSpot=TractorInformation(ii,2);TempTractorStartTime=TractorInformation(ii,3);
        TempTractorDispatchTime=DTT{TractorStartSpot,StartSpot,1};
        EarliestTimeToThisSpot=TempTractorDispatchTime+TempTractorStartTime;
        TempDispathForAllDeckTractor(ii,1)=ii;
        TempDispathForAllDeckTractor(ii,2)=EarliestTimeToThisSpot;
        TempDispathForAllDeckTractor(ii,3)=TempTractorDispatchTime;
    end
    % On time: reaches StartSpot no later than 5 time units after the job start.
    ArriveBeforeJobStartTime=TempDispathForAllDeckTractor(TempDispathForAllDeckTractor(:,2)<=StartTime+5,:);
    % Among on-time tractors pick the least travel time; otherwise earliest arrival.
    if ~isempty(ArriveBeforeJobStartTime)
        SelectedTractor=ArriveBeforeJobStartTime(ArriveBeforeJobStartTime(:,3)==min(ArriveBeforeJobStartTime(:,3)),:);
        TractorID=SelectedTractor(1,1);
        TractorDispatchTime=SelectedTractor(1,3);
        TractorEndTime=StartTime;
        TractorStartTime=TractorEndTime-TractorDispatchTime;
    else
        SelectedTractor=TempDispathForAllDeckTractor(TempDispathForAllDeckTractor(:,2)==min(TempDispathForAllDeckTractor(:,2)),:);
        TractorID=SelectedTractor(1,1);
        TractorEndTime=SelectedTractor(1,2);
        TractorDispatchTime=SelectedTractor(1,3);
        TractorStartTime=TractorEndTime-TractorDispatchTime;
    end

    TractorStartSpot=TractorInformation(TractorID,2);
    TractorEndSpot=StartSpot;
    % Collision-check the tractor's own move: trajectory variant 1, entity type 3.
    [~,~,TractorStartTime,TractorEndTime]=CollisionAvoidance(AllDispatchSchedule,TractorStartSpot,TractorEndSpot,1,3,TractorStartTime);
    % Pre-positioning row: tractor-only move (job type 0, variant 1) that reuses
    % the aircraft job's JobID and JobPriority.
    TempNewDispatchSchedule=[TractorStartSpot,TractorEndSpot,TractorStartTime,TractorEndTime,3,0,JobID,TractorID,1,JobPriority];
    NewDispatchSchedule=[NewDispatchSchedule;TempNewDispatchSchedule];
    AllDispatchSchedule=[AllDispatchSchedule;TempNewDispatchSchedule];
    StartTime=max(TractorEndTime,StartTime);
end
% Route the aircraft; the collision check may delay the returned start time.
[~,~,StartTime,JobEndTime]=CollisionAvoidance(AllDispatchSchedule,StartSpot,EndSpot,TrajectoryPriority,EntityType,StartTime);
TempNewDispatchSchedule=[StartSpot,EndSpot,StartTime,JobEndTime,EntityType,JobType,JobID,TractorID,TrajectoryPriority,JobPriority];
NewDispatchSchedule=[NewDispatchSchedule;TempNewDispatchSchedule];
AllDispatchSchedule=[AllDispatchSchedule;TempNewDispatchSchedule];
% The chosen tractor now stands at EndSpot, free from JobEndTime onward.
TractorInformation(TractorID,2:3)=[EndSpot,JobEndTime];
end
