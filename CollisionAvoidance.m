% Delay a candidate movement until its swept path clears every already-scheduled
% job in AllDispatchSchedule; return the resulting interval and accumulated delay.
%
% Inputs : EntityType - 1 traction system, 2 aircraft taxiing, 3 tractor only.
% Outputs: AllCollisionFree - 1 once no scheduled job conflicts; TotalDelayedTime -
%          accumulated delay; StartTime/EndTime - the final, possibly delayed interval.
%
% Notes: TimeLine is a trajectory's per-step time series, so its last entry is the
% total traversal time; AllDispatchSchedule columns read here are 1-2 spots, 3-4
% times, 5 entity type, 9 trajectory priority.
function [AllCollisionFree,TotalDelayedTime,StartTime,EndTime]=CollisionAvoidance(AllDispatchSchedule,StartSpot,EndSpot,TrajPriority,EntityType,StartTime)
global TLTS TLAT TLT
global vehicle_geometrics_ aircraft_geometrics_
NumOfDispatch=size(AllDispatchSchedule,1);
AllCollisionFree=0;
TotalDelayedTime=0;
% Candidate movement: trajectory and collision footprint for its entity type.
Present.Type=EntityType;
if Present.Type==1
    Present.x=TLTS{StartSpot,EndSpot,TrajPriority}{1,1};
    Present.y=TLTS{StartSpot,EndSpot,TrajPriority}{1,2};
    Present.theta=TLTS{StartSpot,EndSpot,TrajPriority}{1,3};
    Present.TimeLine=TLTS{StartSpot,EndSpot,TrajPriority}{1,4};
    Present.SafeDistance=aircraft_geometrics_.radius;
    Present.Lf=aircraft_geometrics_.Lf;
    Present.Lr=aircraft_geometrics_.Lr;
elseif Present.Type==2
    Present.x=TLAT{StartSpot,EndSpot,TrajPriority}{1,1};
    Present.y=TLAT{StartSpot,EndSpot,TrajPriority}{1,2};
    Present.theta=TLAT{StartSpot,EndSpot,TrajPriority}{1,3};
    Present.TimeLine=TLAT{StartSpot,EndSpot,TrajPriority}{1,4};
    Present.SafeDistance=aircraft_geometrics_.radius;
    Present.Lf=aircraft_geometrics_.Lf;
    Present.Lr=aircraft_geometrics_.Lr;
elseif Present.Type==3
    Present.x=TLT{StartSpot,EndSpot,TrajPriority}{1,1};
    Present.y=TLT{StartSpot,EndSpot,TrajPriority}{1,2};
    Present.theta=TLT{StartSpot,EndSpot,TrajPriority}{1,3};
    Present.TimeLine=TLT{StartSpot,EndSpot,TrajPriority}{1,4};
    Present.SafeDistance=vehicle_geometrics_.radius;
    Present.Lf=vehicle_geometrics_.Lf;
    Present.Lr=vehicle_geometrics_.Lr;
end
Present.StartTime=StartTime;
Present.EndTime=StartTime+Present.TimeLine(end);
if NumOfDispatch==0
    AllCollisionFree=1;
    TotalDelayedTime=0;
end

while AllCollisionFree==0
    for lll=1:NumOfDispatch
        Other.StartTime=AllDispatchSchedule(lll,3);
        Other.EndTime=AllDispatchSchedule(lll,4);
        Other.Type=AllDispatchSchedule(lll,5);
        % Disjoint intervals: no delay needed for this job.
        if Present.StartTime>=Other.EndTime||Present.EndTime<=Other.StartTime
            DelayedTime=0;
        else
            if Other.Type==1
                Other.x=TLTS{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,1};
                Other.y=TLTS{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,2};
                Other.theta=TLTS{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,3};
                Other.TimeLine=TLTS{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,4};
                Other.SafeDistance=aircraft_geometrics_.radius;
                Other.Lf=aircraft_geometrics_.Lf;
                Other.Lr=aircraft_geometrics_.Lr;
            elseif Other.Type==2
                Other.x=TLAT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,1};
                Other.y=TLAT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,2};
                Other.theta=TLAT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,3};
                Other.TimeLine=TLAT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,4};
                Other.SafeDistance=aircraft_geometrics_.radius;
                Other.Lf=aircraft_geometrics_.Lf;
                Other.Lr=aircraft_geometrics_.Lr;
            elseif Other.Type==3
                Other.x=TLT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,1};
                Other.y=TLT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,2};
                Other.theta=TLT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,3};
                Other.TimeLine=TLT{AllDispatchSchedule(lll,1),AllDispatchSchedule(lll,2),AllDispatchSchedule(lll,9)}{1,4};
                Other.SafeDistance=vehicle_geometrics_.radius;
                Other.Lf=vehicle_geometrics_.Lf;
                Other.Lr=vehicle_geometrics_.Lr;
            end
            DelayedTime=CaculateDelayedTime(Present,Other);
            % Shift the candidate, then rescan from the first job: the new interval
            % may conflict with jobs already checked.
            TotalDelayedTime=TotalDelayedTime+DelayedTime;
            Present.StartTime=Present.StartTime+DelayedTime;
            Present.EndTime=Present.EndTime+DelayedTime;
            if DelayedTime~=0
                AllCollisionFree=0;
                break
            end
        end
        % A clean pass through the last job means the movement is clear.
        if lll==NumOfDispatch&&DelayedTime==0
            AllCollisionFree=1;
        end
    end
end
StartTime=Present.StartTime;
EndTime=Present.EndTime;
end
