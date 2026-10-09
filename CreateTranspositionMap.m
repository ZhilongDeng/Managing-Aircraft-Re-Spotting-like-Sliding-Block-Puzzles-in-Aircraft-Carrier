% CreateTranspositionMap
% Move generator for the re-spotting A* search: for each spot, the spots
% reachable in a single transfer along a stored trajectory. Occupancy is
% ignored here; the caller (SpotTransposition) filters occupied destinations.
%
% Notes:
%   - Only the first OLTS variant is scanned, so a pair whose trajectory
%     exists only under a later variant is not recorded.
%   - TranspositionMap is preallocated NumOfSpot x 1 but stored with {1,ii}, so
%     it grows to NumOfSpot x NumOfSpot with only row 1 filled.
function []=CreateTranspositionMap()
global SpotInfo TranspositionMap
load('TrajLibrary.mat')
NumOfSpot=size(SpotInfo,1);
TranspositionMap=cell(NumOfSpot,1);
for ii=1:NumOfSpot
    TempMap=[];
   for jj=1:NumOfSpot
       % Empty entry means no variant-1 trajectory is stored for this pair.
       Obstructive=OLTS{ii,jj,1};
       if ~isempty(Obstructive)
           TempMap=[TempMap,jj];
       end
   end
   TranspositionMap{1,ii}=TempMap;
end
end


