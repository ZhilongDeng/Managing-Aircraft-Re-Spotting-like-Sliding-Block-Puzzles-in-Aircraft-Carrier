% PlotTransferPath - draw the layout sequence produced by the A* transfer
% search: one panel per layout, with the move that produced it drawn over the
% hangar boundary.
%
% LayoutTransfer - one row per layout, starting with the initial layout (one more
%                  row than NodeTransfer); a column index is an aircraft number.
% NodeTransfer   - one row per move: start spot, end spot, trajectory priority.
%                  Row k turns layout k into layout k+1, drawn in panel k+1.
%
% Panels are laid out 3 per row, filled row-wise; spare panels stay empty.

function PlotTransferPath(LayoutTransfer,NodeTransfer)
global SpotInfo
figure
NumOfTransfer=size(LayoutTransfer,1);
aa=3;
bb=ceil(NumOfTransfer/aa);
ha = tight_subplot(bb,aa,[.005 .005],[.005 .005],[.005 .005]);
global TLTS
for ii=1:aa*bb
    axes(ha(ii))
    % Green hangar-boundary outline, drawn on every panel as a reference frame.
    coutour_xy_.x=[    22.00,    100.50,    100.50,    124.20,    124.20,    174.80,    174.80,    198.50,    198.50,   230.00,    230.00,    59.50,    59.50,    35.80,    35.80,    22.00,    22.00];
    coutour_xy_.y=[    20.00,     20.00,      0.00,      0.00,     20.00,     20.00,      0.00,      0.00,     20.00,    20.00,     53.00,    53.00,    74.00,    74.00,    53.00,    53.00,    20.00];
    plot(coutour_xy_.x,coutour_xy_.y,'g-','Linewidth',2)
    hh=fill(coutour_xy_.x,coutour_xy_.y,[0.6 0.6 0.6]);
    set(hh,'edgealpha',0,'facealpha',0.3)
    hold on

    if ii<=NumOfTransfer
        SpotOccupancy=LayoutTransfer(ii,:);
        Nobs=length(SpotOccupancy);
        for jj=1:Nobs
            EndSpot=0;
            if ii>1
                EndSpot=NodeTransfer(ii-1,2);
            end

            % Draw every aircraft except the one being transferred, which
            % occupies EndSpot (0 on panel 1, so nothing is excluded there).
            if SpotOccupancy(jj)~=EndSpot
                [obstacle_vertexes_.x,obstacle_vertexes_.y]=AircraftContour(SpotInfo(SpotOccupancy(jj),:));
                fill(obstacle_vertexes_.x, obstacle_vertexes_.y, [125, 125, 125] ./ 255);
                hold on
                txt=[int2str(SpotOccupancy(jj))];
                % Labels sit midway between contour vertices 1 and 12.
                text((obstacle_vertexes_.x(1)+obstacle_vertexes_.x(12))/2,...
                    (obstacle_vertexes_.y(1)+obstacle_vertexes_.y(12))/2,txt,'HorizontalAlignment','center','FontSize',10)
            else
                TransferAircraft=jj;
            end
        end

        % From panel 2 on, overlay the move itself: its planned trajectory
        % plus the aircraft at its start (red) and end (blue) spots.
        if ii>1
            StartSpot=NodeTransfer(ii-1,1);
            EndSpot=NodeTransfer(ii-1,2);
            TrajPriority=NodeTransfer(ii-1,3);
            Trajx=TLTS{StartSpot,EndSpot,TrajPriority}{1,1};
            Trajy=TLTS{StartSpot,EndSpot,TrajPriority}{1,2};
            plot(Trajx,Trajy,'r','LineWidth',2)

            [obstacle_vertexes_.x,obstacle_vertexes_.y]=AircraftContour(SpotInfo(StartSpot,:));
            hh=fill(obstacle_vertexes_.x, obstacle_vertexes_.y, [225, 0, 0] ./ 255);
            set(hh,'edgealpha',0,'facealpha',0.5)
            hold on
            txt=[int2str(TransferAircraft),'-',int2str(StartSpot)];
            text((obstacle_vertexes_.x(1)+obstacle_vertexes_.x(12))/2,...
                (obstacle_vertexes_.y(1)+obstacle_vertexes_.y(12))/2,txt,'HorizontalAlignment','center','FontSize',10)

            [obstacle_vertexes_.x,obstacle_vertexes_.y]=AircraftContour(SpotInfo(EndSpot,:));
            hh=fill(obstacle_vertexes_.x, obstacle_vertexes_.y, [0, 0, 225] ./ 255);
            set(hh,'edgealpha',0,'facealpha',0.5)
            txt=[int2str(EndSpot)];
            text((obstacle_vertexes_.x(1)+obstacle_vertexes_.x(12))/2,...
                (obstacle_vertexes_.y(1)+obstacle_vertexes_.y(12))/2,txt,'HorizontalAlignment','center','FontSize',10)
        end
    end

    axis equal
    box on
    % Fix the view window so every panel shares the same frame.
    environment_scale_.environment_x_min=0;
    environment_scale_.environment_x_max=250;
    environment_scale_.environment_y_min=0-8;
    environment_scale_.environment_y_max=79+8;
    environment_scale_.x_scale=environment_scale_.environment_x_max-environment_scale_.environment_x_min;
    environment_scale_.y_scale=environment_scale_.environment_y_max-environment_scale_.environment_y_min;
    axis([environment_scale_.environment_x_min environment_scale_.environment_x_max...
        environment_scale_.environment_y_min environment_scale_.environment_y_max])

    % 0:0:0 is empty, so no tick marks are drawn.
    t=0:0:0;
    set(gca,'xtick',t);
    set(gca,'ytick',t);
end
end


function [ha, pos] = tight_subplot(Nh, Nw, gap, marg_h, marg_w)
% tight_subplot - create subplot axes with adjustable gaps and margins.
%
% Nh/Nw are the numbers of subplot rows/columns. gap, marg_h and marg_w are
% in normalized units (0...1) and each accepts a scalar or a two-element
% pair (vertical/horizontal gap, lower/upper margin, left/right margin);
% omitted arguments use the defaults. ha is ordered row-wise from the upper
% left corner, as in subplot.
%
% Attribution: Pekka Kumpulainen 21.5.2012   @tut.fi
% Tampere University of Technology / Automation Science and Engineering
if nargin<3; gap = .02; end
if nargin<4 || isempty(marg_h); marg_h = .05; end
if nargin<5; marg_w = .05; end
if numel(gap)==1;
    gap = [gap gap];
end
if numel(marg_w)==1;
    marg_w = [marg_w marg_w];
end
if numel(marg_h)==1;
    marg_h = [marg_h marg_h];
end
axh = (1-sum(marg_h)-(Nh-1)*gap(1))/Nh;
axw = (1-sum(marg_w)-(Nw-1)*gap(2))/Nw;
py = 1-marg_h(2)-axh;
ii = 0;
for ih = 1:Nh
    px = marg_w(1);

    for ix = 1:Nw
        ii = ii+1;
        ha(ii) = axes('Units','normalized', ...
            'Position',[px py axw axh], ...
            'XTickLabel','', ...
            'YTickLabel','');
        px = px+axw+gap(2);
    end
    py = py-axh-gap(1);
end
if nargout > 1
    pos = get(ha,'Position');
end
ha = ha(:);
end

function [x,y] = AircraftContour(Coordinate)
% AircraftContour - outline polygon of a single aircraft, placed on the deck.
% Coordinate is one SpotInfo row [x, y, heading] (heading in radians); x, y are
% the closed outline vertices in deck coordinates. The outline is defined in
% the aircraft's local frame with the nose toward +y; the heading is reduced by
% 90 degrees to rotate it into the deck angle convention, then translated.
xm=Coordinate(1,1);
ym=Coordinate(1,2);
tm=Coordinate(1,3);
x=[-0.00 0.35 0.50 0.50 1.64 1.90 1.94 4.45 4.43 1.35 1.79 1.79 3.49 3.49 3.02 1.00 0.81 0.66 -0.66 -0.81 -1.00 -3.03 -3.49 -3.49 -1.79 -1.79 -1.35 -4.21 -4.45 -4.45 -1.94 -1.90 -1.64 -0.50 -0.50 -0.35 -0.02 -0.00];
y=[11.75 11.14 10.66 7.57 4.80 3.43 2.66 1.22 -1.68 -1.81 -2.91 -3.36 -5.20 -6.23 -6.55 -5.44 -5.44 -5.66 -5.66 -5.44 -5.44 -6.55 -6.20 -5.20 -3.39 -2.91 -1.81 -1.71 -1.62 1.18 2.66 3.40 4.80 7.57 10.66 11.14 11.73 11.75];
tm=tm-0.5*pi;
x2=x.*cos(tm)-y.*sin(tm);
y2=x.*sin(tm)+y.*cos(tm);
x=x2+xm;
y=y2+ym;
end
