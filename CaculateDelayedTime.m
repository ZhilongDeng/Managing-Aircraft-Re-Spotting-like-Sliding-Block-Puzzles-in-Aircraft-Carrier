% CaculateDelayedTime - delay (s) keeping two time-overlapping movements a safe
% distance apart.
%
% Inputs:  Present, Other - movements with fields Type, StartTime, EndTime,
%          x, y (sampled trajectory) and SafeDistance.
% Outputs: DelayedTime - seconds the caller adds to Present's StartTime and
%          EndTime; 0 when the two movements never conflict.
% Notes:   Type 1 = traction system, 2 = aircraft taxiing, 3 = tractor only.
%          Present is a local copy: the shifts below are invisible to the caller.

function DelayedTime=CaculateDelayedTime(Present,Other)
global TimeInterval 
CollisionFree=0;
DelayedTime=0;
TotalSafeDistan=(Present.SafeDistance+Other.SafeDistance)^2;
% Both units are traction systems: a movement at rest keeps occupying its end
% point, so Present's end position is tested against all of Other's path first.
if Present.Type==1&&Other.Type==1
    % No +1 here: one sample short of the movement's final sample.
    PresentEndx=Present.x(floor((Present.EndTime-Present.StartTime)/TimeInterval));
    PresentEndy=Present.y(floor((Present.EndTime-Present.StartTime)/TimeInterval));
    OtherAllx=Other.x(1:floor((Other.EndTime-Other.StartTime)/TimeInterval));
    OtherAlly=Other.y(1:floor((Other.EndTime-Other.StartTime)/TimeInterval));
    Distan2=(OtherAllx-PresentEndx).^2+(OtherAlly-PresentEndy).^2;
    if min(Distan2)<=TotalSafeDistan
        % Postpone Present until Other's closest approach to that position
        % coincides with Present.EndTime; an earlier one needs no delay.
        TempIndex=find(Distan2==min(Distan2));
        if Other.StartTime+TempIndex*TimeInterval-Present.EndTime>=0
            DelayedTime=DelayedTime+(Other.StartTime+TempIndex*TimeInterval-Present.EndTime);
        end
        Present.StartTime=Present.StartTime+DelayedTime;
        Present.EndTime=Present.EndTime+DelayedTime;
    end
end

% Shift Present forward one TimeInterval per pass until every pair of
% simultaneously occupied samples is a safe distance apart.
while CollisionFree==0
    % Overlap of the two time windows; StartforOverlap is measured from each
    % movement's own StartTime and bounds the trajectory slice to test.
    if Present.StartTime<=Other.StartTime&&Present.EndTime>=Other.EndTime
        Other.StartforOverlap=0;
        Present.StartforOverlap=Other.StartTime-Present.StartTime;
        OverlapTimeLength=Other.EndTime-Other.StartTime;
    elseif Present.StartTime>=Other.StartTime&&Present.EndTime<=Other.EndTime
        Other.StartforOverlap=Present.StartTime-Other.StartTime;
        Present.StartforOverlap=0;
        OverlapTimeLength=Present.EndTime-Present.StartTime;
    elseif Present.StartTime<=Other.StartTime&&Present.EndTime<=Other.EndTime
        Other.StartforOverlap=0;
        Present.StartforOverlap=Other.StartTime-Present.StartTime;
        OverlapTimeLength=Present.EndTime-Other.StartTime;
    elseif Present.StartTime>=Other.StartTime&&Present.EndTime>=Other.EndTime
        Other.StartforOverlap=Present.StartTime-Other.StartTime;
        Present.StartforOverlap=0;
        OverlapTimeLength=Other.EndTime-Present.StartTime;
    end
    % Complete TimeInterval steps inside the overlap; an overlap shorter than
    % one step contains no sample and is collision free.
    NumOfNodes=floor(OverlapTimeLength/TimeInterval);
    if NumOfNodes<=0
        DelayedTime=DelayedTime+0;
        CollisionFree=1;
    else
        % The +1 converts each time offset into a 1-based sample index.
        Otherx=Other.x(floor(Other.StartforOverlap/TimeInterval)+1:floor(Other.StartforOverlap/TimeInterval)+NumOfNodes);
        Othery=Other.y(floor(Other.StartforOverlap/TimeInterval)+1:floor(Other.StartforOverlap/TimeInterval)+NumOfNodes);
        Presentx=Present.x(floor(Present.StartforOverlap/TimeInterval)+1:floor(Present.StartforOverlap/TimeInterval)+NumOfNodes);
        Presenty=Present.y(floor(Present.StartforOverlap/TimeInterval)+1:floor(Present.StartforOverlap/TimeInterval)+NumOfNodes);
        Distan=(Otherx-Presentx).^2+(Othery-Presenty).^2;
        if min(Distan)>=TotalSafeDistan
            CollisionFree=1;
        else
            DelayedTime=DelayedTime+TimeInterval;
            Present.StartTime=Present.StartTime+TimeInterval;
            Present.EndTime=Present.EndTime+TimeInterval;
        end
    end
end
end
