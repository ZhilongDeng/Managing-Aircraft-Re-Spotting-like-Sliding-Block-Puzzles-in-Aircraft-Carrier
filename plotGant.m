% plotGant  Draw the re-spotting Gantt chart for a completed schedule: one
% horizontal bar per activity, arranged in aircraft, tractor and elevator bands
% with the fill pattern encoding the entity type.
%
% Inputs:  Schedule       - dispatch schedule (DT), one activity per row of the
%                           10-column layout (spot pair, times, entity type,
%                           aircraft, tractor).
%          SpotsOccupancy - spot-occupancy matrix; only its width is read.
% Note:    Colours index the 10x3 mycolor palette by column 8; elevator rows
%          carry 0 there, so all elevator bars share one band row labelled "Elevator0".
function plotGant(Schedule,SpotsOccupancy)
% Drop activities of 5 s or less; they are too short to be readable.
aa=find(Schedule(:,4)-Schedule(:,3)<=5);
Schedule(aa,:)=[];
NumOfDipatch=size(Schedule,1);
NumOfJob=size(SpotsOccupancy,2);
load(['180color','.mat'],'mycolor');

% Tractor numbers double as palette rows; a +4 shift for hangar destinations
% (spots 137-219) gives hangar tractors their own colour range.
for ii=1:size(Schedule,1)
    SpotInHanger=[137:219];
    IsInHanger=~isempty(intersect(Schedule(ii,2),SpotInHanger));
    if IsInHanger&&(Schedule(ii,5)==1||Schedule(ii,5)==3)
        Schedule(ii,8)= Schedule(ii,8)+4;
    end
end

% Split the schedule into three bands, each sorted by resource then start time.
% Entity 1 (traction system) appears in both the aircraft and tractor bands.
AircraftSchedule=Schedule(Schedule(:,5)==1|Schedule(:,5)==5,:);
AircraftSchedule=sortrows(AircraftSchedule,[7 3],{'ascend' 'ascend'});
TractorSchedule=Schedule(Schedule(:,5)==3|Schedule(:,5)==1,:);
TractorSchedule=sortrows(TractorSchedule,[8 3],{'ascend' 'ascend'});
ElevatorSchedule=Schedule(Schedule(:,5)==4|Schedule(:,5)==5,:);
ElevatorSchedule=sortrows(ElevatorSchedule,[8 3],{'ascend' 'ascend'});

% Tick positions and resource labels are collected while drawing and de-duplicated below.
TickRec=[];
LaybleRec={};

figure;
% Aircraft band. Entity 1 (traction system) draws 'cross'; entity 5 (elevator
% carrying an aircraft) draws 'circles'.
NumOfAicraftSchedule=size(AircraftSchedule,1);
AicraftLable=unique(AircraftSchedule(:,7));
NumOfAicraft=size(AicraftLable,1);
for i=1:NumOfAicraftSchedule
    StartSpot=AircraftSchedule(i,1);
    EndSpot=AircraftSchedule(i,2);
    aa=find(AicraftLable==AircraftSchedule(i,7));
    x=[AircraftSchedule(i,3),AircraftSchedule(i,4),AircraftSchedule(i,4),AircraftSchedule(i,3),AircraftSchedule(i,3)];
    y=[aa/2+0.23,aa/2+0.23,aa/2-0.23,aa/2-0.23,aa/2+0.23];
    if AircraftSchedule(i,5)==5
        PlotColor=mycolor(AircraftSchedule(i,8)+8,:);  % entity 5: column 8 is 0; +8 selects palette row 8
    else
        PlotColor=mycolor(AircraftSchedule(i,8),:);
    end
    hold on
    plot(x,y,'Color',PlotColor,LineWidth=0.5)
    hold on
    hh=fill(x,y,PlotColor);
    set(hh,'edgealpha',0,'facealpha',0.3)
    if AircraftSchedule(i,5)==1
        hold on
        fillPolygonPattern(x, y, 'cross', PlotColor, -3,1, 0.15);
    elseif AircraftSchedule(i,5)==5
        hold on
        fillPolygonPattern(x, y, 'circles', PlotColor, 4, (x(2)-x(1))/3, (y(2)-y(3))/0.08);
    end
    txt=[int2str(StartSpot),'-',int2str(EndSpot)];
    hold on
    text((x(1)+x(2))/2,(y(3)+y(2))/2,txt,'HorizontalAlignment','center')
    TickRec=[TickRec,(y(3)+y(2))/2];
    LaybleRec=horzcat(LaybleRec, ['Aircraft',num2str(AircraftSchedule(i,7))]);
end

% Tractor band, offset NumOfAicraft rows down. Entity 1 (traction system) draws
% 'cross'; entity 3 (tractor only) draws 'hatch'.
NumOfTractorSchedule=size(TractorSchedule,1);
TractorLable=unique(TractorSchedule(:,8));
NumOfTractor=size(TractorLable,1);
for i=1:NumOfTractorSchedule
    StartSpot=TractorSchedule(i,1);
    EndSpot=TractorSchedule(i,2);
    aa=find(TractorLable==TractorSchedule(i,8));
    x=[TractorSchedule(i,3),TractorSchedule(i,4),TractorSchedule(i,4),TractorSchedule(i,3),TractorSchedule(i,3)];
    y=[(aa+NumOfAicraft)/2+0.23,(aa+NumOfAicraft)/2+0.23,(aa+NumOfAicraft)/2-0.23,(aa+NumOfAicraft)/2-0.23,(aa+NumOfAicraft)/2+0.23];
    PlotColor=mycolor(TractorSchedule(i,8),:);
    hold on
    plot(x,y,'Color',PlotColor,LineWidth=0.5)
    hold on
    hh=fill(x,y,PlotColor);
    set(hh,'edgealpha',0,'facealpha',0.3)
    if TractorSchedule(i,5)==1
        hold on
        fillPolygonPattern(x, y, 'cross', PlotColor, -3,1, 0.15);
    elseif TractorSchedule(i,5)==3
        hold on
        fillPolygonPattern(x, y, 'hatch',  PlotColor, -3,1, 0.15);
    end
    txt=[int2str(StartSpot),'-',int2str(EndSpot)];
    hold on
    text((x(1)+x(2))/2,(y(3)+y(2))/2,txt,'HorizontalAlignment','center')
    TickRec=[TickRec,(y(3)+y(2))/2];
    LaybleRec=horzcat(LaybleRec, ['Tractor',num2str(TractorSchedule(i,8))]);
end

% Elevator band, offset below the other two bands. Entity 4 (travelling empty)
% draws 'dots'; entity 5 (carrying an aircraft) draws 'circles'.
NumOfElevatorSchedule=size(ElevatorSchedule,1);
ElevatorLable=unique(ElevatorSchedule(:,8));
for i=1:NumOfElevatorSchedule
    StartSpot=ElevatorSchedule(i,1);
    EndSpot=ElevatorSchedule(i,2);
    aa=find(ElevatorLable==ElevatorSchedule(i,8));
    x=[ElevatorSchedule(i,3),ElevatorSchedule(i,4),ElevatorSchedule(i,4),ElevatorSchedule(i,3),ElevatorSchedule(i,3)];
    y=[(aa+NumOfAicraft+NumOfTractor)/2+0.23,(aa+NumOfAicraft+NumOfTractor)/2+0.23,(aa+NumOfAicraft+NumOfTractor)/2-0.23,(aa+NumOfAicraft+NumOfTractor)/2-0.23,(aa+NumOfAicraft+NumOfTractor)/2+0.23];
    PlotColor=mycolor(ElevatorSchedule(i,8)+8,:);
    hold on
    plot(x,y,'Color',PlotColor,LineWidth=0.5)
    hold on
    hh=fill(x,y,PlotColor);
    set(hh,'edgealpha',0,'facealpha',0.3)
    if ElevatorSchedule(i,5)==4
        hold on
        fillPolygonPattern(x, y, 'dots',  PlotColor, 8, (x(2)-x(1))/3, (y(2)-y(3))/0.08);
    elseif ElevatorSchedule(i,5)==5
        hold on
        fillPolygonPattern(x, y, 'circles', PlotColor, 4, (x(2)-x(1))/3, (y(2)-y(3))/0.08);
    end
    txt=[int2str(StartSpot),'-',int2str(EndSpot)];
    hold on
    text((x(1)+x(2))/2,(y(3)+y(2))/2,txt,'HorizontalAlignment','center')
    TickRec=[TickRec,(y(3)+y(2))/2];
    LaybleRec=horzcat(LaybleRec, ['Elevator',num2str(ElevatorSchedule(i,8))]);
end
TickRec = unique(TickRec, 'stable');
LaybleRec = unique(LaybleRec, 'stable');

% Bar centres become the y ticks; tick labels are drawn by hand below.
set(gca, 'YTick', TickRec, 'YTickLabel', []);

% Resource names, colour-coded by band, just outside the y axis.
hold on;
xl = xlim;
xpos = xl(1)-2;
for i = 1:length(TickRec)
    ypos = TickRec(i);
    label = LaybleRec{i};

    if contains(label, 'Tractor')
        textColor = [0 0.4470 0.7410];   % blue
    elseif contains(label, 'Elevator')
        textColor = [0.8500 0.3250 0.0980];   % orange
    else
        textColor = [0.4660 0.6740 0.1880];   % green (aircraft)
    end

    text(xpos, ypos, label, ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment', 'middle', ...
        'Color', textColor, ...
        'FontSize', get(gca, 'FontSize'));
end
hold off;

xlabel('t/s');
set(gca, 'TickLength', [0 0]);   % Hide the tick marks.
end


function fillPolygonPattern(x, y, style, varargin)
% fillPolygonPattern  Fill a polygon with a solid or patterned fill.
%
%   x, y : polygon vertices; the outline is closed automatically.
%   Trailing style-specific arguments (defaults in the parse helpers below):
%       'solid'            color
%       'hatch', 'cross'   color, angle in degrees, lineWidth, spacing
%       'hatch-left/right' lineColor, lineWidth, spacing
%       'dots'             dotColor, dotSize, grid density (nx, or [nx ny])
%       'circles'          circleColor, circleSize, grid density, lineWidth
%   A missing or non-positive spacing uses the bounding-box diagonal / 30.
%   'dots' and 'circles' skip points on the boundary; the outline is not drawn.

% Work with vertex columns so the geometry below is orientation-independent.
x = x(:);
y = y(:);

if length(x) < 3
    error('多边形至少需要三个顶点');
end

switch lower(style)
    case 'solid'
        if nargin < 4
            error('实心填充需要提供颜色参数');
        end
        color = varargin{1};
        fill(x, y, color, 'EdgeColor', 'none');

    case 'hatch'
        [color, angle, lineWidth, spacing] = parseHatchAngleArgs(varargin, x, y);
        theta = angle * pi/180;
        drawHatchInPolygon(x, y, theta, color, lineWidth, spacing);

    case 'cross'
        [color, angle, lineWidth, spacing] = parseHatchAngleArgs(varargin, x, y);
        theta1 = angle * pi/180;
        theta2 = -angle * pi/180;
        drawHatchInPolygon(x, y, theta1, color, lineWidth, spacing);
        drawHatchInPolygon(x, y, theta2, color, lineWidth, spacing);

    case 'hatch-left'
        [lineColor, lineWidth, spacing] = parseFixedHatchArgs(varargin, x, y);
        theta = 45 * pi/180;
        drawHatchInPolygon(x, y, theta, lineColor, lineWidth, spacing);

    case 'hatch-right'
        [lineColor, lineWidth, spacing] = parseFixedHatchArgs(varargin, x, y);
        theta = -45 * pi/180;
        drawHatchInPolygon(x, y, theta, lineColor, lineWidth, spacing);

    case 'dots'
        [dotColor, dotSize, nx, ny] = parseDotsArgs(varargin);
        drawDotsInPolygon(x, y, dotColor, dotSize, nx, ny, 'filled');

    case 'circles'
        [circleColor, circleSize, nx, ny, lineWidth] = parseCirclesArgs(varargin);
        drawDotsInPolygon(x, y, circleColor, circleSize, nx, ny, 'hollow', lineWidth);

    otherwise
        error('未知的填充样式：%s，可选 solid, hatch, cross, hatch-left, hatch-right, dots, circles', style);
end
end

% ---------- Argument-parsing helpers ----------

function [lineColor, lineWidth, spacing] = parseFixedHatchArgs(args, x, y)
lineColor = 'k';
lineWidth = 1;
spacing = [];

nArgs = numel(args);
if nArgs >= 1 && ~isempty(args{1})
    lineColor = args{1};
end
if nArgs >= 2 && ~isempty(args{2})
    lineWidth = args{2};
end
if nArgs >= 3 && ~isempty(args{3})
    spacing = args{3};
end

if isempty(spacing) || spacing <= 0
    spacing = defaultSpacing(x, y);
end
end

function [color, angle, lineWidth, spacing] = parseHatchAngleArgs(args, x, y)
color = 'k';
angle = 45;
lineWidth = 1;
spacing = [];

nArgs = numel(args);
if nArgs >= 1 && ~isempty(args{1})
    color = args{1};
end
if nArgs >= 2 && ~isempty(args{2})
    angle = args{2};
end
if nArgs >= 3 && ~isempty(args{3})
    lineWidth = args{3};
end
if nArgs >= 4 && ~isempty(args{4})
    spacing = args{4};
end

if isempty(spacing) || spacing <= 0
    spacing = defaultSpacing(x, y);
end
end

function [dotColor, dotSize, nx, ny] = parseDotsArgs(args)
dotColor = 'k';
dotSize = 6;
nx = 15;
ny = 15;

nArgs = numel(args);
if nArgs >= 1 && ~isempty(args{1})
    dotColor = args{1};
end
if nArgs >= 2 && ~isempty(args{2})
    dotSize = args{2};
end
if nArgs >= 3 && ~isempty(args{3})
    if nArgs == 3
        nx = args{3};
        ny = nx;
    else
        nx = args{3};
        ny = args{4};
    end
end
end

function [circleColor, circleSize, nx, ny, lineWidth] = parseCirclesArgs(args)
circleColor = 'k';
circleSize = 6;
nx = 15;
ny = 15;
lineWidth = 1;

nArgs = numel(args);
if nArgs >= 1 && ~isempty(args{1})
    circleColor = args{1};
end
if nArgs >= 2 && ~isempty(args{2})
    circleSize = args{2};
end
if nArgs >= 3 && ~isempty(args{3})
    if nArgs == 3
        nx = args{3};
        ny = nx;
    elseif nArgs == 4
        nx = args{3};
        ny = args{4};
    elseif nArgs >= 5
        nx = args{3};
        ny = args{4};
        lineWidth = args{5};
    end
end
end

function sp = defaultSpacing(x, y)
% Spacing used when callers pass none or a non-positive value: one thirtieth
% of the bounding-box diagonal, or 1 for a degenerate box.
xmin = min(x); xmax = max(x);
ymin = min(y); ymax = max(y);
diagLen = sqrt((xmax-xmin)^2 + (ymax-ymin)^2);
if diagLen == 0
    sp = 1;
else
    sp = diagLen / 30;
end
end

% ---------- Drawing core ----------

function drawHatchInPolygon(x, y, theta, lineColor, lineWidth, spacing)
% Draw parallel lines at angle theta, clipped to the polygon; spacing is the
% perpendicular distance between neighbouring lines.
scanDir = [-sin(theta), cos(theta)];  % Unit normal to the lines.

xmin = min(x); xmax = max(x);
ymin = min(y); ymax = max(y);

corners = [xmin ymin; xmin ymax; xmax ymin; xmax ymax];
proj = corners * scanDir';
projMin = min(proj);
projMax = max(proj);

% One scan line per spacing step across the polygon's projected extent.
projValues = projMin : spacing : projMax;
% Add a final line if the step misses the far edge.
if projValues(end) < projMax - 1e-12
    projValues = [projValues, projMax];
end

holdState = ishold;
hold on;

% Closed edge list, one row per edge as [x1 y1 x2 y2]; the last vertex closes
% back to the first.
edges = [x, y, x([2:end,1]), y([2:end,1])];
nEdges = size(edges, 1);

for p = projValues
    intersections = [];
    for e = 1:nEdges
        x1 = edges(e,1); y1 = edges(e,2);
        x2 = edges(e,3); y2 = edges(e,4);

        proj1 = [x1, y1] * scanDir';
        proj2 = [x2, y2] * scanDir';

        % A sign change of the projection means the edge crosses this line.
        if (proj1 - p) * (proj2 - p) <= 0
            if abs(proj2 - proj1) < eps
                continue;
            end
            t = (p - proj1) / (proj2 - proj1);
            t = max(0, min(1, t));
            interX = x1 + t * (x2 - x1);
            interY = y1 + t * (y2 - y1);
            intersections = [intersections; interX, interY];
        end
    end

    % Round before de-duplicating so a vertex hit twice collapses to one point.
    if size(intersections, 1) >= 2
        intersections = unique(round(intersections, 10), 'rows');
    end

    % Crossings come in pairs: after sorting along the line, consecutive pairs
    % bound each interior span.
    if size(intersections, 1) >= 2
        lineDir = [cos(theta), sin(theta)];
        projOnLine = intersections * lineDir';
        [~, idx] = sort(projOnLine);
        sortedPts = intersections(idx, :);

        nPts = size(sortedPts, 1);
        for k = 1:2:nPts-1
            plot(sortedPts([k, k+1], 1), sortedPts([k, k+1], 2), ...
                'Color', lineColor, 'LineWidth', lineWidth);
        end
    end
end

if ~holdState
    hold off;
end
end

function drawDotsInPolygon(x, y, dotColor, dotSize, nx, ny, markerType, lineWidth)
% Keep only grid points strictly inside the polygon; boundary points are dropped.
xmin = min(x); xmax = max(x);
ymin = min(y); ymax = max(y);

xg = linspace(xmin, xmax, nx);
yg = linspace(ymin, ymax, ny);
[X, Y] = meshgrid(xg, yg);

[in, on] = inpolygon(X, Y, x, y);
inStrict = in & ~on;

hold on;
if strcmp(markerType, 'filled')
    plot(X(inStrict), Y(inStrict), '.', 'Color', dotColor, 'MarkerSize', dotSize);
else
    plot(X(inStrict), Y(inStrict), 'o', 'Color', dotColor, 'MarkerSize', dotSize, 'LineWidth', lineWidth);
end
hold off;
end
