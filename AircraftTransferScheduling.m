% AircraftTransferScheduling - serial scheduler for one re-spotting turn. Walks the
% ordered job list from the A* search and, for each job, determines its type, tests
% it for obstruction, assigns an elevator and a tractor, and appends timed rows.
%
% Job rows (JobList in, TransferSchedule out) are N-by-10: 1 StartSpot, 2 EndSpot,
% 3 StartTime, 4 EndTime, 5 EntityType, 6 JobType, 7 JobID, 8 TractorID,
% 9 TrajectoryPriority, 10 JobPriority; TransferSchedule column 5 also uses
% 4 = elevator travelling empty, 5 = elevator carrying an aircraft.
% Notes: JobType (column 6) selects the branch taken: 1-4 deck to deck, 5-8 hangar
% to hangar, 9-12 hangar to deck, 13-16 deck to hangar, 17 standalone elevator move.
function [TransferSchedule]=AircraftTransferScheduling(JobList)
%% Globals and turn-local working copies
global DTTS OLTS DTT % DTTS/OLTS: traction-system dispatch-time and obstacle-list libraries
global ElevatorSpot
global AllDeckDispatchSchedule AllHangerDispatchSchedule AllElevatorSchedule
global ElevatorInfo DeckTractorInfo HangerTractorInfo
global SpotsOccupancy SpotOnDeck SpotInHanger
AllDeckDispatchScheduleInTnisTurn=AllDeckDispatchSchedule;
AllHangerDispatchScheduleInTnisTurn=AllHangerDispatchSchedule;
AllElevatorScheduleInTnisTurn=AllElevatorSchedule;
SpotOccupancyInThisTurn=SpotsOccupancy;
DeckTractorInformationInThisTurn=DeckTractorInfo;% tractor status tables: 1 number, 2 current spot, 3 earliest available time
HangerTractorInformationInThisTurn=HangerTractorInfo;
ElevatorSpotInthisTurn=ElevatorSpot;% hand-off spots: row 1 deck, row 2 hangar
ElevatorInformationInThisturn=ElevatorInfo;% elevator status: same three columns
TransferSchedule=[];
%% Schedule the jobs one at a time
% Order jobs by assigned tractor (column 8) and set each job's type code (column 6)
JobList=sortrows(JobList,8);
JobList=UpDateJobList(JobList);
num=1;
while ~isempty(JobList)
    CurrentJob=JobList(1,:);
    StartSpot=CurrentJob(1,1);
    EndSpot=CurrentJob(1,2);
    StartTime=CurrentJob(1,3);
    EndTime=CurrentJob(1,4);
    EntityType=CurrentJob(1,5);
    JobType=CurrentJob(1,6);
    JobID=CurrentJob(1,7);
    TractorID=CurrentJob(1,8);
    TrajectoryPriority=CurrentJob(1,9);
    JobPriority=CurrentJob(1,10);
    %% Test the job for obstruction and derive its scheduling legs
    % Single-layer job: one feasibility test covers the whole move
    if JobType<=8
        [IsObstructive,TrajectoryPriority]=JudgeDispatchInfo(StartSpot,EndSpot,EntityType,SpotOccupancyInThisTurn);
    elseif JobType<=16
        % Two-stage job: Stage1/Stage2 are the origin and destination layers (1 = deck, 2 = hangar)
        if 8<JobType&&JobType<=12;Stage1=2;Stage2=1;
        elseif 12<JobType&&JobType<=16;Stage1=1;Stage2=2;
        end
        [IsObstructive,TrajectoryPriorityInStage1,TrajectoryPriorityInStage2,ElevatorID]=...
            JudgeTransferInfo(StartSpot,EndSpot,EntityType,SpotOccupancyInThisTurn,Stage1,Stage2,ElevatorSpotInthisTurn,ElevatorInformationInThisturn);
        StartSpotForStage1=StartSpot;
        EndSpotForStage1=ElevatorSpotInthisTurn(Stage1,ElevatorID);
        StartSpotForStage2=ElevatorSpotInthisTurn(Stage2,ElevatorID);
        EndSpotForStage2=EndSpot;
    elseif JobType==17
        % Standalone elevator move: vertical travel cannot be obstructed, so fix the elevator and the two levels directly
        IsObstructive=0;
        [rowStart,colStart]=find(ElevatorSpotInthisTurn==StartSpot);
        [rowEnd,~]        =find(ElevatorSpotInthisTurn==EndSpot);
        ElevatorID=colStart(1);
        StartLevel=rowStart(1);
        TargetLevel=rowEnd(1);
    end
    if IsObstructive;num=num+1;break;end   % fully obstructed: end the pass, returning the schedule built so far
    %% Schedule the job: pick resources, then emit timed legs
    % Deck-to-deck job (types 1-4)
    if 0<JobType&&JobType<=4
        AllDispatchSchedule=[AllDeckDispatchSchedule;AllHangerDispatchSchedule];
        if ~isempty(AllDispatchSchedule)
            AllAircraftDispatchSchedule=AllDispatchSchedule(AllDispatchSchedule(:,5)==1,:);
            % A leg already scheduled blocks this route if its start spot lies in this
            % route's obstacle list, or this job's end spot lies in the other leg's
            % obstacle list (or is that leg's end spot); the job then waits for it to start.
            for ii=1:size(AllAircraftDispatchSchedule,1)
                TempObstructiveList1=OLTS{StartSpot,EndSpot,TrajectoryPriority};
                TempIsObstructiveList1=intersect(TempObstructiveList1,AllAircraftDispatchSchedule(ii,1));

                TempObstructiveList2=OLTS{AllAircraftDispatchSchedule(ii,1),AllAircraftDispatchSchedule(ii,2),AllAircraftDispatchSchedule(ii,9)};
                TempIsObstructiveList2=intersect([TempObstructiveList2,AllAircraftDispatchSchedule(ii,2)],EndSpot);
                if ~isempty(TempIsObstructiveList1)||~isempty(TempIsObstructiveList2)
                    StartTime=max(StartTime,AllAircraftDispatchSchedule(ii,3));
                end
            end
        end

        Obstructive=OLTS{StartSpot,EndSpot,TrajectoryPriority};
        NeedElevator=intersect(Obstructive,ElevatorSpot(1,:));
        if ~isempty(NeedElevator)
            ElevatorID=find(ElevatorSpot(1,:)==NeedElevator);
            if ElevatorInformationInThisturn(ElevatorID,2)~=NeedElevator
                [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
                    (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,1,ElevatorInformationInThisturn);
                NewDispatchSchedule(end,5)=4;
                TransferSchedule=[TransferSchedule;NewDispatchSchedule];
                StartTime=NewDispatchSchedule(end,4);
            end
        end
        DispatchJob=[StartSpot,EndSpot,StartTime,EndTime,EntityType,JobType,JobID,TractorID,TrajectoryPriority,JobPriority];
        [NewDispatchSchedule,AllDeckDispatchScheduleInTnisTurn,DeckTractorInformationInThisTurn]=PLanningDispatchSchedule...
            (AllDeckDispatchScheduleInTnisTurn,DispatchJob,DeckTractorInformationInThisTurn);
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        % Hangar-to-hangar job (types 5-8)
    elseif 4<JobType&&JobType<=8
         AllDispatchSchedule=[AllDeckDispatchSchedule;AllHangerDispatchSchedule];
        if ~isempty(AllDispatchSchedule)
            AllAircraftDispatchSchedule=AllDispatchSchedule(AllDispatchSchedule(:,5)==1,:);
            for ii=1:size(AllAircraftDispatchSchedule,1)
                TempObstructiveList1=OLTS{StartSpot,EndSpot,TrajectoryPriority};
                TempIsObstructiveList1=intersect(TempObstructiveList1,AllAircraftDispatchSchedule(ii,1));

                TempObstructiveList2=OLTS{AllAircraftDispatchSchedule(ii,1),AllAircraftDispatchSchedule(ii,2),AllAircraftDispatchSchedule(ii,9)};
                TempIsObstructiveList2=intersect([TempObstructiveList2,AllAircraftDispatchSchedule(ii,2)],EndSpot);
                if ~isempty(TempIsObstructiveList1)||~isempty(TempIsObstructiveList2)
                    StartTime=max(StartTime,AllAircraftDispatchSchedule(ii,3));
                end
            end
        end

        Obstructive=OLTS{StartSpot,EndSpot,TrajectoryPriority};
        NeedElevator=intersect([Obstructive,EndSpot],ElevatorSpot(2,:));
        if ~isempty(NeedElevator)
            ElevatorID=find(ElevatorSpot(2,:)==NeedElevator);
            if ElevatorInformationInThisturn(ElevatorID,2)~=NeedElevator
                [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
                    (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,2,ElevatorInformationInThisturn);
                NewDispatchSchedule(end,5)=4;
                TransferSchedule=[TransferSchedule;NewDispatchSchedule];
                StartTime=NewDispatchSchedule(end,4);
            end
        end
        DispatchJob=[StartSpot,EndSpot,StartTime,EndTime,EntityType,JobType,JobID,TractorID,TrajectoryPriority,JobPriority];
        [NewDispatchSchedule,AllHangerDispatchScheduleInTnisTurn,HangerTractorInformationInThisTurn]=PLanningDispatchSchedule...
            (AllHangerDispatchScheduleInTnisTurn,DispatchJob,HangerTractorInformationInThisTurn);
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        % Hangar-to-deck job (types 9-12): hangar leg, then elevator up, then deck leg
    elseif 8<JobType&&JobType<=12
        AllDispatchSchedule=[AllDeckDispatchSchedule;AllHangerDispatchSchedule];
        if ~isempty(AllDispatchSchedule)
            AllAircraftDispatchSchedule=AllDispatchSchedule(AllDispatchSchedule(:,5)==1,:);
            for ii=1:size(AllAircraftDispatchSchedule,1)
                TempObstructiveList1=OLTS{StartSpotForStage1,EndSpotForStage1,TrajectoryPriorityInStage1};
                TempIsObstructiveList1=intersect(TempObstructiveList1,AllAircraftDispatchSchedule(ii,1));
                TempObstructiveList2=OLTS{AllAircraftDispatchSchedule(ii,1),AllAircraftDispatchSchedule(ii,2),AllAircraftDispatchSchedule(ii,9)};
                TempIsObstructiveList2=intersect([TempObstructiveList2,AllAircraftDispatchSchedule(ii,2)],EndSpot);
                if ~isempty(TempIsObstructiveList1)||~isempty(TempIsObstructiveList2)
                    StartTime=max(StartTime,AllAircraftDispatchSchedule(ii,3));
                end
            end
        end
        % Stage 1: move the elevator empty to the hand-off spot (column 5 = 4),
        % then run the hangar leg up to it
        [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
            (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,Stage1,ElevatorInformationInThisturn);
        NewDispatchSchedule(end,5)=4;
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        % Stage-1 timing runs backwards from the elevator's arrival: the leg ends
        % at the later of the elevator becoming ready and StartTime+DispatchTime
        DispatchTimeForStage1=DTTS{StartSpotForStage1,EndSpotForStage1,TrajectoryPriorityInStage1};
        EndTimeForStage1=max(AllElevatorScheduleInTnisTurn(end,4),StartTime+DispatchTimeForStage1);
        StartTimeForStage1=EndTimeForStage1-DispatchTimeForStage1;
        DispatchJob=[StartSpotForStage1,EndSpotForStage1,StartTimeForStage1,EndTimeForStage1,EntityType,JobType,JobID,TractorID,TrajectoryPriorityInStage1,JobPriority];
        [NewDispatchSchedule,AllHangerDispatchScheduleInTnisTurn,HangerTractorInformationInThisTurn]=PLanningDispatchSchedule...
            (AllHangerDispatchScheduleInTnisTurn,DispatchJob,HangerTractorInformationInThisTurn);
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        % The elevator is free once the hangar leg reaches it
        ElevatorInformationInThisturn(ElevatorID,3)=AllHangerDispatchScheduleInTnisTurn(end,4);


        AllDispatchSchedule=[AllDeckDispatchSchedule;AllHangerDispatchSchedule];
        if ~isempty(AllDispatchSchedule)
            AllAircraftDispatchSchedule=AllDispatchSchedule(AllDispatchSchedule(:,5)==1,:);
            for ii=1:size(AllAircraftDispatchSchedule,1)
                TempObstructiveList1=OLTS{StartSpotForStage2,EndSpotForStage2,TrajectoryPriorityInStage2};
                TempIsObstructiveList1=intersect(TempObstructiveList1,AllAircraftDispatchSchedule(ii,1));
                TempObstructiveList2=OLTS{AllAircraftDispatchSchedule(ii,1),AllAircraftDispatchSchedule(ii,2),AllAircraftDispatchSchedule(ii,9)};
                TempIsObstructiveList2=intersect([TempObstructiveList2,AllAircraftDispatchSchedule(ii,2)],EndSpot);
                if ~isempty(TempIsObstructiveList1)||~isempty(TempIsObstructiveList2)
                    StartTime=max(StartTime,AllAircraftDispatchSchedule(ii,3));
                end
            end
        end
        % Stage 2: the elevator carries the aircraft up to the deck (column 5 = 5),
        % then the deck leg runs from the hand-off spot to the destination
        [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
            (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,Stage2,ElevatorInformationInThisturn);
        NewDispatchSchedule(end,5)=5;
        % Stage-2 timing follows the elevator directly: it starts when the lift ends
        DispatchTimeForStage2=DTTS{StartSpotForStage2,EndSpotForStage2,TrajectoryPriorityInStage2};
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        StartTimeForStage2=AllElevatorScheduleInTnisTurn(end,4);
        EndTimeForStage2=StartTimeForStage2+DispatchTimeForStage2;
        DispatchJob=[StartSpotForStage2,EndSpotForStage2,StartTimeForStage2,EndTimeForStage2,EntityType,JobType,JobID,TractorID,TrajectoryPriorityInStage2,JobPriority];
        [NewDispatchSchedule,AllDeckDispatchScheduleInTnisTurn,DeckTractorInformationInThisTurn]=PLanningDispatchSchedule...
            (AllDeckDispatchScheduleInTnisTurn,DispatchJob,DeckTractorInformationInThisTurn);
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        ElevatorInformationInThisturn(ElevatorID,3)=AllDeckDispatchScheduleInTnisTurn(end,3);
        % Deck-to-hangar job (types 13-16): deck leg, then elevator down, then hangar leg
    elseif 12<JobType&&JobType<=16

        AllDispatchSchedule=[AllDeckDispatchSchedule;AllHangerDispatchSchedule];
        if ~isempty(AllDispatchSchedule)
            AllAircraftDispatchSchedule=AllDispatchSchedule(AllDispatchSchedule(:,5)==1,:);
            for ii=1:size(AllAircraftDispatchSchedule,1)
                TempObstructiveList1=OLTS{StartSpotForStage1,EndSpotForStage1,TrajectoryPriorityInStage1};
                TempIsObstructiveList1=intersect(TempObstructiveList1,AllAircraftDispatchSchedule(ii,1));
                TempObstructiveList2=OLTS{AllAircraftDispatchSchedule(ii,1),AllAircraftDispatchSchedule(ii,2),AllAircraftDispatchSchedule(ii,9)};
                TempIsObstructiveList2=intersect([TempObstructiveList2,AllAircraftDispatchSchedule(ii,2)],EndSpot);
                if ~isempty(TempIsObstructiveList1)||~isempty(TempIsObstructiveList2)
                    StartTime=max(StartTime,AllAircraftDispatchSchedule(ii,3));
                end
            end
        end
        % Stage 1: move the elevator empty to the hand-off spot (column 5 = 4),
        % then run the deck leg up to it
        [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
            (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,Stage1,ElevatorInformationInThisturn);
        NewDispatchSchedule(end,5)=4;
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        % Stage-1 timing runs backwards from the elevator's arrival: the leg ends
        % at the later of the elevator becoming ready and StartTime+DispatchTime
        DispatchTimeForStage1=DTTS{StartSpotForStage1,EndSpotForStage1,TrajectoryPriorityInStage1};
        EndTimeForStage1=max(AllElevatorScheduleInTnisTurn(end,4),StartTime+DispatchTimeForStage1);
        StartTimeForStage1=EndTimeForStage1-DispatchTimeForStage1;
        DispatchJob=[StartSpotForStage1,EndSpotForStage1,StartTimeForStage1,EndTimeForStage1,EntityType,JobType,JobID,TractorID,TrajectoryPriorityInStage1,JobPriority];
        [NewDispatchSchedule,AllDeckDispatchScheduleInTnisTurn,DeckTractorInformationInThisTurn]=PLanningDispatchSchedule...
            (AllDeckDispatchScheduleInTnisTurn,DispatchJob,DeckTractorInformationInThisTurn);
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        % The elevator is free once the deck leg reaches it
        ElevatorInformationInThisturn(ElevatorID,3)=AllDeckDispatchScheduleInTnisTurn(end,4);



        AllDispatchSchedule=[AllDeckDispatchSchedule;AllHangerDispatchSchedule];
        if ~isempty(AllDispatchSchedule)
            AllAircraftDispatchSchedule=AllDispatchSchedule(AllDispatchSchedule(:,5)==1,:);
            for ii=1:size(AllAircraftDispatchSchedule,1)
                TempObstructiveList1=OLTS{StartSpotForStage2,EndSpotForStage2,TrajectoryPriorityInStage2};
                TempIsObstructiveList1=intersect(TempObstructiveList1,AllAircraftDispatchSchedule(ii,1));
                TempObstructiveList2=OLTS{AllAircraftDispatchSchedule(ii,1),AllAircraftDispatchSchedule(ii,2),AllAircraftDispatchSchedule(ii,9)};
                TempIsObstructiveList2=intersect([TempObstructiveList2,AllAircraftDispatchSchedule(ii,2)],EndSpot);
                if ~isempty(TempIsObstructiveList1)||~isempty(TempIsObstructiveList2)
                    StartTime=max(StartTime,AllAircraftDispatchSchedule(ii,3));
                end
            end
        end
        % Stage 2: the elevator carries the aircraft down to the hangar (column 5 = 5),
        % then the hangar leg runs from the hand-off spot to the destination
        [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
            (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,Stage2,ElevatorInformationInThisturn);
        NewDispatchSchedule(end,5)=5;
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        % Stage-2 timing follows the elevator directly: it starts when the lift ends
        DispatchTimeForStage2=DTTS{StartSpotForStage2,EndSpotForStage2,TrajectoryPriorityInStage2};
        StartTimeForStage2=AllElevatorScheduleInTnisTurn(end,4);
        EndTimeForStage2=StartTimeForStage2+DispatchTimeForStage2;
        DispatchJob=[StartSpotForStage2,EndSpotForStage2,StartTimeForStage2,EndTimeForStage2,EntityType,JobType,JobID,TractorID,TrajectoryPriorityInStage2,JobPriority];
        [NewDispatchSchedule,AllHangerDispatchScheduleInTnisTurn,HangerTractorInformationInThisTurn]=PLanningDispatchSchedule...
            (AllHangerDispatchScheduleInTnisTurn,DispatchJob,HangerTractorInformationInThisTurn);
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];
        ElevatorInformationInThisturn(ElevatorID,3)=AllHangerDispatchScheduleInTnisTurn(end,3);

        % Standalone elevator move (job 17): pre-position empty, then carry the aircraft
    elseif JobType==17
        % Stage A: pre-position the elevator at StartSpot; a zero-length move takes no time
        ElevatorInformationInThisturn(ElevatorID,3)=max(ElevatorInformationInThisturn(ElevatorID,3),StartTime);
        [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
            (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,StartLevel,ElevatorInformationInThisturn);
        if NewDispatchSchedule(end,4)>NewDispatchSchedule(end,3)
            % Mark the empty-elevator code only when the elevator actually moves
            NewDispatchSchedule(end,5)=4;
        end
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];

        % Stage B: carry the aircraft across to TargetLevel
        ElevatorInformationInThisturn(ElevatorID,3)=AllElevatorScheduleInTnisTurn(end,4);
        [NewDispatchSchedule,AllElevatorScheduleInTnisTurn,ElevatorInformationInThisturn]=PLanningElevatorSchedule...
            (AllElevatorScheduleInTnisTurn,CurrentJob,ElevatorID,TargetLevel,ElevatorInformationInThisturn);
        NewDispatchSchedule(end,5)=5;
        TransferSchedule=[TransferSchedule;NewDispatchSchedule];

        % The elevator is free once the aircraft is unloaded
        ElevatorInformationInThisturn(ElevatorID,3)=AllElevatorScheduleInTnisTurn(end,4);
    end
    % The aircraft has moved: record its new spot in the working occupancy
    TempAircraftNumber=find(SpotOccupancyInThisTurn==JobList(num,1));
    SpotOccupancyInThisTurn(TempAircraftNumber)=JobList(num,2);


    % Chain the aircraft's next job: a job starting at this transfer's end spot waits for this transfer to end
    if ~isempty(TransferSchedule)
        AircraftNextJob=find(JobList(:,1)==TransferSchedule(end,2));
        if ~isempty(AircraftNextJob)
            JobList(AircraftNextJob(1),3)=TransferSchedule(end,4);
        end
    end



    JobList(num,:)=[];
    num=1;
    AllDeckDispatchSchedule=AllDeckDispatchScheduleInTnisTurn;
    AllHangerDispatchSchedule=AllHangerDispatchScheduleInTnisTurn;
    AllElevatorSchedule=AllElevatorScheduleInTnisTurn;
    SpotsOccupancy=SpotOccupancyInThisTurn;
    DeckTractorInfo=DeckTractorInformationInThisTurn;
    HangerTractorInfo=HangerTractorInformationInThisTurn;
    ElevatorInfo=ElevatorInformationInThisturn;

end
end
