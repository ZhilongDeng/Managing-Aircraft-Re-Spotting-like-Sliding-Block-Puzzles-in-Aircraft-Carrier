% Central configuration for aircraft re-spotting: loads the trajectory
% library and spot geometry, defines the spot classes and the elevator /
% tractor resources, and sets the A* weights, timing precision, prohibited
% periods and vehicle geometry.
%
% Call once at start-up, before any scheduling routine; TrajLibrary.mat and
% SpotInfo.mat must be on the path.
function []=InitTransferParameters()

% From TrajLibrary.mat: TL = trajectories, DT = dispatch times, OL = obstacle
% lists; suffix TS = traction system, AT = aircraft taxiing, T = tractor.
global TLTS TLAT TLT
global DTTS DTAT DTT
global OLTS OLAT OLT
load('TrajLibrary.mat','TLTS','TLAT','TLT','DTTS','DTAT','DTT','OLTS','OLAT','OLT')
% SpotInfo: one row per spot; columns 1-3 give the aircraft pose (x, y,
% theta), column 4 the tractor heading.
global SpotInfo
load('SpotInfo.mat','SpotInfo')
% One numbering for all spots; SpotOnDeck and SpotInHanger give the deck and
% hangar ranges.
global SpotOnDeck SpotInHanger
global ElevatorSpot
global DeckTractorSpot HangerTractorSpot
global FixedWing1SpotsOnDeck FixedWing2SpotsOnDeck Helicopter1SpotsOnDeck Helicopter2SpotsOnDeck 
global FixedWing1SpotsInHanger FixedWing2SpotsInHanger Helicopter1SpotsInHanger Helicopter2SpotsInHanger
SpotOnDeck=[1:136];
SpotInHanger=[137:219];
ElevatorSpot=[55,54,53;177,178,179];% row 1 deck spots, row 2 hangar spots
DeckTractorSpot=[1 2 3 4];
HangerTractorSpot=[137 138 139 140];
% Spot ranges per aircraft class; UpDateJobList reads them to classify jobs.
FixedWing1SpotsOnDeck=[1:76];
FixedWing2SpotsOnDeck=[77:107];
Helicopter1SpotsOnDeck=[108:128];
Helicopter2SpotsOnDeck=[129:136];
FixedWing1SpotsInHanger=[137:179];
FixedWing2SpotsInHanger=[];
Helicopter1SpotsInHanger=[180:219];
Helicopter2SpotsInHanger=[];

% Dispatch schedules, reset so each run starts from an empty schedule.
global AllDeckDispatchSchedule AllHangerDispatchSchedule
AllDeckDispatchSchedule=[];
AllHangerDispatchSchedule=[];

% Transfer resources; each row is [index, current spot, available time].
global ElevatorInfo DeckTractorInfo HangerTractorInfo
ElevatorInfo=[1 177 0;2 178 0;3 179 0];
DeckTractorInfo=[1 26 0;2 27 0;3 28 0;4 31 0 ];
HangerTractorInfo=[1 137 0;2 138 0;3 139 0;4 140 0];

global SpotsOccupancy % set by the caller before this function runs

% Prohibited intervals, [start end] per row.
global ProhibitedPeriods
ProhibitedPeriods=[0 80;100 120];

% Sampling step of the stored trajectories, used by the collision check.
global TimeInterval
TimeInterval=0.1;

% Duration of one elevator trip between levels.
global ElevatorTransferTime
ElevatorTransferTime=100;


% A* weights for hangar re-spotting: f = g + multiplier_H * h; multiplier_Posi
% scales the distance-based transfer-time estimate used when the dispatch table
% has no entry; the *_Times_Penalty terms charge extra moves and level crossings.
global  Astar_
Astar_.multiplier_H=5;
Astar_.multiplier_Posi=3;
Astar_.Transfer_Times_Penalty=100;
Astar_.CrossLayer_Times_Penalty=100;
Astar_.max_iter=5000;

% Spot-to-spot move graph for the A* search; entry i lists the spots an
% aircraft at spot i may move to.
global  TranspositionMap
CreateTranspositionMap();

% Footprints for the collision model, in metres: length = wheelbase + front_hang
% + rear_hang, radius is the circumscribed radius used as the safe distance, and
% Lf / Lr are the front / rear longitudinal offsets from the reference point.
global vehicle_geometrics_ aircraft_geometrics_
vehicle_geometrics_.wheelbase=1.8;
vehicle_geometrics_.front_hang=0.7;
vehicle_geometrics_.rear_hang=1;
vehicle_geometrics_.width=1.8;
vehicle_geometrics_.length=vehicle_geometrics_.wheelbase+...
    vehicle_geometrics_.front_hang+vehicle_geometrics_.rear_hang;
vehicle_geometrics_.radius=sqrt((vehicle_geometrics_.length/4)^2+(vehicle_geometrics_.width/2)^2);
vehicle_geometrics_.Lr=1/4*vehicle_geometrics_.length-vehicle_geometrics_.rear_hang;
vehicle_geometrics_.Lf=3/4*vehicle_geometrics_.length-vehicle_geometrics_.rear_hang;

aircraft_geometrics_.wheelbase=8;
aircraft_geometrics_.front_hang=4;
aircraft_geometrics_.rear_hang=6.3;
aircraft_geometrics_.width=9.94;
aircraft_geometrics_.length=aircraft_geometrics_.wheelbase+...
    aircraft_geometrics_.front_hang+aircraft_geometrics_.rear_hang;
aircraft_geometrics_.radius=sqrt((aircraft_geometrics_.length/4)^2+(aircraft_geometrics_.width/2)^2);
aircraft_geometrics_.Lr=1/4*aircraft_geometrics_.length-aircraft_geometrics_.rear_hang;
aircraft_geometrics_.Lf=3/4*aircraft_geometrics_.length-aircraft_geometrics_.rear_hang;

end