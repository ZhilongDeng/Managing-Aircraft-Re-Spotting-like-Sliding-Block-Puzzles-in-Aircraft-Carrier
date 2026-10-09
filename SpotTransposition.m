%% SPOTTRANSPOSITION  Re-spot a whole air wing by A* search over deck/hangar layouts.
% A layout maps each aircraft slot to the spot it occupies; a move slides one
% aircraft to a free, reachable spot (sliding-block puzzle). Returns the chain
% of single-aircraft transfers; the flag says whether the target was reached.
% Inputs: StartArrangement/EndArrangement - current/target spot of each moved
%   aircraft, in matching order; InitialLayout - spot of each aircraft slot.
% Outputs: NodeTransfer N-by-3 [startSpot endSpot trajectoryPriority];
%   AircraftNumber N-by-1 mover slot; TransferCompletenessFlag 1 if reached.
% Notes: the load('TrajLibrary.mat') below creates local copies, not globals;
%   CalculateH and TrajectorySelcet read the global DTTS/OLTS instead.
function [NodeTransfer,AircraftNumber,TransferCompletenessFlag]=SpotTransposition(StartArrangement,EndArrangement,InitialLayout)
global SpotInHanger SpotOnDeck
global SpotInfo OLTSWoT
global  Astar_ TranspositionMap
%% Load pre-computed search data
load('TrajLibrary.mat')
load('SpotInfo.mat')
%% Search for a transfer path, then plot it
[NodeTransfer,LayoutTransfer,AircraftNumber,TransferCompletenessFlag,iter] = ...
    SearchTransferPath(StartArrangement,EndArrangement,InitialLayout);
TransferCompletenessFlag
PlotTransferPath(LayoutTransfer,NodeTransfer)
end


function [NodeTransfer,LayoutTransfer,AircraftNumber,CompletenessFlag,iter] = SearchTransferPath(StartArrangement,EndArrangement,InitialLayout)
% SearchTransferPath  A* search over whole-layout states. Openlist/NodePool are
% pre-allocated arrays; layouts are keyed by a char hash for O(1) lookup.
global SpotInHanger SpotOnDeck ElevatorSpot
global Astar_ TranspositionMap

nAircraft = size(InitialLayout,2);

% Terminal layout: for each requested move, put its target spot at the slot
% that currently holds its start spot.
TerminalLayout = InitialLayout;
for ii = 1:size(StartArrangement,1)
    Posi = find(InitialLayout == StartArrangement(ii));
    TerminalLayout(Posi) = EndArrangement(ii);
end

%% Pre-computed lookup tables, indexed directly by spot ID
% Largest spot ID anywhere, so the guards below can index the tables by ID
allSpotIDs = [InitialLayout(:); SpotInHanger(:); SpotOnDeck(:); ElevatorSpot(:)];
maxSpotID  = max(allSpotIDs);

% IsSpotInHanger(sp) : true if spot sp is inside a hangar
IsSpotInHanger = false(1, maxSpotID);
IsSpotInHanger(SpotInHanger) = true;

% ElevatorPartner(sp) : the spot at the other end of sp's elevator, 0 when sp is
% not an elevator. ElevatorSpot row 1 holds the deck spots, row 2 the hangar ones.
ElevatorPartner = zeros(1, maxSpotID);
for ii = 1:size(ElevatorSpot, 2)
    ElevatorPartner(ElevatorSpot(1, ii)) = ElevatorSpot(2, ii);
    ElevatorPartner(ElevatorSpot(2, ii)) = ElevatorSpot(1, ii);
end

% sprintf format that turns a layout into its hash key, e.g. '%d,%d,' -> '5,12,'
keyFormat = [repmat('%d,', 1, nAircraft)];

%% Initial node
InitNode_InitialLayout    = InitialLayout;
InitNode_g                = 0;
InitNode_h                = CalculateH(InitialLayout, TerminalLayout);
InitNode_f                = InitNode_g + Astar_.multiplier_H * InitNode_h;
InitNode_IsInOpenList     = 1;
InitNode_IsInCloseList    = 0;
InitNode_ParentLayout     = -999 * ones(1, nAircraft);   % sentinel for backtracking
InitNode_TrajectoryPriority = 0;

BestEverVal = Inf;
BestEverInd = 1;

%% Pre-allocate Openlist & NodePool
% Initial capacity guess, not a true bound: one expansion can add more than
% nAircraft children, since a single aircraft may have several reachable spots.
% ExpandStorage below doubles the arrays whenever this runs out.
max_nodes = Astar_.max_iter * nAircraft + 100;
if max_nodes < 10000
    max_nodes = 10000;
end

% --- Openlist (rows 1:OpenlistCount are active) ---
Openlist.Layout             = zeros(max_nodes, nAircraft);
Openlist.g                  = zeros(max_nodes, 1);
Openlist.h                  = zeros(max_nodes, 1);
Openlist.f                  = zeros(max_nodes, 1);
Openlist.ParentLayout       = zeros(max_nodes, nAircraft);
Openlist.TrajectoryPriority = zeros(max_nodes, 1);
OpenlistCount = 1;

% --- NodePool (rows 1:NodePoolCount store all explored nodes) ---
NodePool.Layout             = zeros(max_nodes, nAircraft);
NodePool.g                  = zeros(max_nodes, 1);
NodePool.h                  = zeros(max_nodes, 1);
NodePool.f                  = zeros(max_nodes, 1);
NodePool.IsInOpenList       = zeros(max_nodes, 1);
NodePool.IsInCloseList      = zeros(max_nodes, 1);
NodePool.ParentLayout       = zeros(max_nodes, nAircraft);
NodePool.TrajectoryPriority = zeros(max_nodes, 1);
NodePoolCount = 1;

% --- NodePoolMap: layout char-key -> NodePool row, for O(1) lookup ---
NodePoolMap = containers.Map('KeyType', 'char', 'ValueType', 'double');

% --- Seed both structures with the initial node ---
initKey = sprintf(keyFormat, InitNode_InitialLayout);

Openlist.Layout(1,:)             = InitNode_InitialLayout;
Openlist.g(1)                    = InitNode_g;
Openlist.h(1)                    = InitNode_h;
Openlist.f(1)                    = InitNode_f;
Openlist.ParentLayout(1,:)       = InitNode_ParentLayout;
Openlist.TrajectoryPriority(1)   = InitNode_TrajectoryPriority;

NodePool.Layout(1,:)             = InitNode_InitialLayout;
NodePool.g(1)                    = InitNode_g;
NodePool.h(1)                    = InitNode_h;
NodePool.f(1)                    = InitNode_f;
NodePool.IsInOpenList(1)         = InitNode_IsInOpenList;
NodePool.IsInCloseList(1)        = InitNode_IsInCloseList;
NodePool.ParentLayout(1,:)       = InitNode_ParentLayout;
NodePool.TrajectoryPriority(1)   = InitNode_TrajectoryPriority;

NodePoolMap(initKey) = 1;

%% A* main loop
iter = 0;
CompletenessFlag = 0;

while (OpenlistCount > 0) && (iter <= Astar_.max_iter) && (~CompletenessFlag)
    iter = iter + 1;

    % --- Take the open node of smallest f, the last one among ties ---
    Temp = min(Openlist.f(1:OpenlistCount));
    CurNodeOrder = find(Openlist.f(1:OpenlistCount) == Temp, 1, 'last');

    Cur_Layout = Openlist.Layout(CurNodeOrder, :);
    Cur_g      = Openlist.g(CurNodeOrder, :);

    % --- Remove it from Openlist by swap-and-pop, O(1) ---
    Openlist.Layout(CurNodeOrder,:)             = Openlist.Layout(OpenlistCount,:);
    Openlist.g(CurNodeOrder,:)                  = Openlist.g(OpenlistCount,:);
    Openlist.h(CurNodeOrder,:)                  = Openlist.h(OpenlistCount,:);
    Openlist.f(CurNodeOrder,:)                  = Openlist.f(OpenlistCount,:);
    Openlist.ParentLayout(CurNodeOrder,:)       = Openlist.ParentLayout(OpenlistCount,:);
    Openlist.TrajectoryPriority(CurNodeOrder,:) = Openlist.TrajectoryPriority(OpenlistCount,:);
    OpenlistCount = OpenlistCount - 1;

    % --- Mark it closed in NodePool ---
    curKey = sprintf(keyFormat, Cur_Layout);
    Ind1 = NodePoolMap(curKey);
    NodePool.IsInOpenList(Ind1)  = 0;
    NodePool.IsInCloseList(Ind1) = 1;

    % --- Occupied-spot bitmask for the elevator checks below ---
    isOccupied = false(1, maxSpotID);
    isOccupied(Cur_Layout) = true;

    % --- Expand cur_node ---
    for jj = 1:nAircraft
        ExplorableSpot = TranspositionMap{1, Cur_Layout(jj)};
        % Drop already-occupied spots (faster than setdiff: no sorting)
        ExplorableSpot = ExplorableSpot(~ismember(ExplorableSpot, Cur_Layout));
        if isempty(ExplorableSpot)
            continue
        end

        for kk = 1:length(ExplorableSpot)
            Child_Layout = Cur_Layout;
            Child_Layout(jj) = ExplorableSpot(kk);

            % --- Elevator entry is blocked while the partner spot is occupied,
            %     unless that occupant is the aircraft itself ---
            childPartner = ElevatorPartner(ExplorableSpot(kk));
            if childPartner > 0
                if isOccupied(childPartner) && (childPartner ~= Cur_Layout(jj))
                    continue
                end
            end

            % --- Is this child already in NodePool? ---
            childKey = sprintf(keyFormat, Child_Layout);
            if NodePoolMap.isKey(childKey)
                Ind2 = NodePoolMap(childKey);
                [IsPassable, TransferTime, TrajectoryPriority] = ...
                    TrajectorySelcet(Cur_Layout, Cur_Layout(jj), Child_Layout(jj));
                if IsPassable
                    % Cross-layer penalty hard-coded as 100 here; the new-child
                    % branch below reads Astar_.CrossLayer_Times_Penalty instead.
                    TransferCost = 0;
                    if Cur_Layout(jj) <= maxSpotID && Child_Layout(jj) <= maxSpotID
                        if IsSpotInHanger(Cur_Layout(jj)) ~= IsSpotInHanger(Child_Layout(jj))
                            TransferCost = 100;
                        end
                    end
                    Child_g = Cur_g + TransferTime + TransferCost + Astar_.Transfer_Times_Penalty;
                    Child_h = CalculateH(Child_Layout, TerminalLayout);
                    Child_f = Child_g + Astar_.multiplier_H * Child_h;

                    % Replace only when f improves by a 0.1 margin, so floating-
                    % point noise does not trigger endless tiny rewrites.
                    if Child_f + 0.1 < NodePool.f(Ind2)
                        if NodePool.IsInOpenList(Ind2) == 1
                            % Still open -> update its Openlist row in place
                            Ind3 = find(sum(abs(Openlist.Layout(1:OpenlistCount,:) - Child_Layout), 2) == 0);
                            Openlist.g(Ind3)                  = Child_g;
                            Openlist.h(Ind3)                  = Child_h;
                            Openlist.f(Ind3)                  = Child_f;
                            Openlist.ParentLayout(Ind3,:)     = Cur_Layout;
                            Openlist.TrajectoryPriority(Ind3) = TrajectoryPriority;
                        else
                            % Already closed -> push a fresh row to re-open it
                            if OpenlistCount >= max_nodes
                                [Openlist, NodePool, max_nodes] = ExpandStorage(Openlist, NodePool, max_nodes);
                            end
                            OpenlistCount = OpenlistCount + 1;
                            Openlist.Layout(OpenlistCount,:)           = Child_Layout;
                            Openlist.g(OpenlistCount)                  = Child_g;
                            Openlist.h(OpenlistCount)                  = Child_h;
                            Openlist.f(OpenlistCount)                  = Child_f;
                            Openlist.ParentLayout(OpenlistCount,:)     = Cur_Layout;
                            Openlist.TrajectoryPriority(OpenlistCount) = TrajectoryPriority;
                        end
                        % NodePool row index is unchanged, so the hash map stays valid
                        NodePool.g(Ind2)                  = Child_g;
                        NodePool.h(Ind2)                  = Child_h;
                        NodePool.f(Ind2)                  = Child_f;
                        NodePool.IsInOpenList(Ind2)       = 1;
                        NodePool.IsInCloseList(Ind2)      = 0;
                        NodePool.ParentLayout(Ind2,:)     = Cur_Layout;
                        NodePool.TrajectoryPriority(Ind2) = TrajectoryPriority;
                    end
                end
                continue
            end

            % --- Child is new (never explored) ---
            [IsPassable, TransferTime, TrajectoryPriority] = ...
                TrajectorySelcet(Cur_Layout, Cur_Layout(jj), Child_Layout(jj));
            if ~IsPassable
                continue
            end

            % Cross-layer penalty (hangar<->deck), from the configured constant
            TransferCost = 0;
            if Cur_Layout(jj) <= maxSpotID && Child_Layout(jj) <= maxSpotID
                if IsSpotInHanger(Cur_Layout(jj)) ~= IsSpotInHanger(Child_Layout(jj))
                    TransferCost = Astar_.CrossLayer_Times_Penalty;
                end
            end

            Child_g = Cur_g + TransferTime + TransferCost + Astar_.Transfer_Times_Penalty;
            Child_h = CalculateH(Child_Layout, TerminalLayout);
            Child_f = Child_g + Astar_.multiplier_H * Child_h;

            % --- Grow the pre-allocated arrays when full ---
            if OpenlistCount >= max_nodes
                [Openlist, NodePool, max_nodes] = ExpandStorage(Openlist, NodePool, max_nodes);
            end

            % --- Add to Openlist ---
            OpenlistCount = OpenlistCount + 1;
            Openlist.Layout(OpenlistCount,:)           = Child_Layout;
            Openlist.g(OpenlistCount)                  = Child_g;
            Openlist.h(OpenlistCount)                  = Child_h;
            Openlist.f(OpenlistCount)                  = Child_f;
            Openlist.ParentLayout(OpenlistCount,:)     = Cur_Layout;
            Openlist.TrajectoryPriority(OpenlistCount) = TrajectoryPriority;

            % --- Add to NodePool ---
            NodePoolCount = NodePoolCount + 1;
            NodePool.Layout(NodePoolCount,:)           = Child_Layout;
            NodePool.g(NodePoolCount)                  = Child_g;
            NodePool.h(NodePoolCount)                  = Child_h;
            NodePool.f(NodePoolCount)                  = Child_f;
            NodePool.IsInOpenList(NodePoolCount)       = 1;
            NodePool.IsInCloseList(NodePoolCount)      = 0;
            NodePool.ParentLayout(NodePoolCount,:)     = Cur_Layout;
            NodePool.TrajectoryPriority(NodePoolCount) = TrajectoryPriority;

            % --- Register in the hash map ---
            NodePoolMap(childKey) = NodePoolCount;

            % --- Track the lowest h seen: it seeds backtracking and is the
            %     best-effort root when the goal is never reached ---
            if Child_h < BestEverVal
                BestEverVal = Child_h;
                BestEverInd = NodePoolCount;
            end

            % --- Goal check ---
            if isequal(Child_Layout, TerminalLayout)
                CompletenessFlag = 1;
                BestEverInd = NodePoolCount;
                break
            end
        end
        if CompletenessFlag
            break
        end
    end
end

%% Backtrack to recover the path
CurBestParentLayout   = NodePool.ParentLayout(BestEverInd, :);
LayoutTransfer        = NodePool.Layout(BestEverInd, :);
TempTrajectoryPriority = NodePool.TrajectoryPriority(BestEverInd, :);
NodeTransfer   = [];
AircraftNumber = [];

% The root stores the -999 sentinel in ParentLayout, which ends the walk
while CurBestParentLayout(1) > -1
    parentKey = sprintf(keyFormat, CurBestParentLayout);
    if NodePoolMap.isKey(parentKey)
        CurInd = NodePoolMap(parentKey);
    else
        break   % safety: should never happen
    end
    CurBestParentLayout = NodePool.ParentLayout(CurInd, :);
    % Prepend the move: a move changes exactly one slot, so setxor(parent,child)
    % is [startSpot endSpot]; the third column is the trajectory priority
    NodeTransfer = [setxor(NodePool.Layout(CurInd,:), LayoutTransfer(1,:), 'stable'), ...
                    TempTrajectoryPriority; ...
                    NodeTransfer];                                         %#ok<AGROW>
    LayoutTransfer = [NodePool.Layout(CurInd,:); LayoutTransfer];          %#ok<AGROW>
    TempTrajectoryPriority = NodePool.TrajectoryPriority(CurInd);
end

% AircraftNumber: the slot that held each move's start spot, i.e. the mover
for ii = 1:size(NodeTransfer, 1)
    TempAircraftNumber = find(LayoutTransfer(ii,:) == NodeTransfer(ii,1));
    AircraftNumber = [AircraftNumber; TempAircraftNumber];                 %#ok<AGROW>
end

end


%% Helper: expand pre-allocated storage
function [Openlist, NodePool, max_nodes] = ExpandStorage(Openlist, NodePool, max_nodes)
% ExpandStorage  Double the row capacity of Openlist and NodePool; existing row
% indices are unchanged, so the hash map stays valid.

new_max = max_nodes * 2;

% Grow each field by assigning past its current end; MATLAB zero-fills the gap
Openlist.Layout(new_max, end)             = 0;
Openlist.g(new_max, 1)                    = 0;
Openlist.h(new_max, 1)                    = 0;
Openlist.f(new_max, 1)                    = 0;
Openlist.ParentLayout(new_max, end)       = 0;
Openlist.TrajectoryPriority(new_max, 1)   = 0;

NodePool.Layout(new_max, end)             = 0;
NodePool.g(new_max, 1)                    = 0;
NodePool.h(new_max, 1)                    = 0;
NodePool.f(new_max, 1)                    = 0;
NodePool.IsInOpenList(new_max, 1)         = 0;
NodePool.IsInCloseList(new_max, 1)        = 0;
NodePool.ParentLayout(new_max, end)       = 0;
NodePool.TrajectoryPriority(new_max, 1)   = 0;

max_nodes = new_max;
end


%% Heuristic function
function val = CalculateH(StartLayout, EndLayout)
% CalculateH  Estimate of the remaining cost: per aircraft, the transfer time
% from its current spot to its target spot (tabulated, else position-based).
%
% Not an admissible lower bound: Astar_.Transfer_Times_Penalty is added for
% every aircraft, including ones already on target, so h does not vanish at the
% goal layout and the search is not guaranteed to return a minimum-cost path.
global SpotInfo Astar_ DTTS SpotInHanger

% Persistent spot->hangar flag table, enlarged when SpotInHanger reaches a
% higher spot ID
persistent IsHanger maxSpot
if isempty(IsHanger) || length(IsHanger) < max(SpotInHanger)
    maxSpot  = max(SpotInHanger);
    IsHanger = false(1, maxSpot);
    IsHanger(SpotInHanger) = true;
end

val = 0;
n   = size(StartLayout, 2);
for ii = 1:n
    startSp = StartLayout(ii);
    endSp   = EndLayout(ii);
    % Transfer time of the first tabulated trajectory option, when one exists
    Transfertime = DTTS{startSp, endSp, 1};
    if isempty(Transfertime)
        if startSp ~= endSp
            StartCfg = SpotInfo(startSp, :);
            EndCfg   = SpotInfo(endSp, :);
            % No tabulated option: estimate from the (x,y) positions
            Transfertime = Astar_.multiplier_Posi * ...
                sqrt((StartCfg(1)-EndCfg(1))^2 + (StartCfg(2)-EndCfg(2))^2) / 2;
            % Cross-layer penalty across the hangar/deck boundary
            if startSp <= maxSpot && endSp <= maxSpot
                if IsHanger(startSp) ~= IsHanger(endSp)
                    Transfertime = Transfertime + Astar_.CrossLayer_Times_Penalty;
                end
            end
        else
            Transfertime = 0;
        end
    end
    val = val + Transfertime + Astar_.Transfer_Times_Penalty;
end
end


%% Trajectory selector
function [val1, val2, val3] = TrajectorySelcet(CurLayout, StartSpot, EndSpot)
% TrajectorySelcet  First trajectory option from StartSpot to EndSpot whose
% obstruction list, plus the destination spot, is clear of CurLayout. Returns
% IsPassable (0 when no option is clear), transfer time, option index.
global OLTS DTTS
val1 = 0;
val2 = 0;
val3 = 0;
NumOfTrajectory = size(OLTS, 3);
for ii = 1:NumOfTrajectory
    Obstructive1 = OLTS{StartSpot, EndSpot, ii};
    if ~isempty(Obstructive1)
        Obstructive = [Obstructive1, EndSpot];
        % The destination spot is appended above, so it must be free as well
        if ~any(ismember(CurLayout, Obstructive))
            val1 = 1;
            val2 = DTTS{StartSpot, EndSpot, ii};
            val3 = ii;
            break
        end
    end
end
end
