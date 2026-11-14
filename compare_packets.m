function results = compare_packets(groundFile, noisyFile, filteredFile)
%COMPARE_PACKETS Plot SR-UKF truth, measurements, estimates, and errors.
% Base MATLAB R2016b+; no toolboxes required.
%
% Put this function and the three text files in the current MATLAB folder:
%   results = compare_packets;
% Or provide explicit paths:
%   results = compare_packets('ground_packets.txt', 'test_packets.txt', ...
%                             'test_packets1_filtered.txt');
%
% Accepted packet formats:
%   G timestamp x y                  (optional fifth numeric field ignored)
%   O timestamp speed heading yaw_rate
%   E timestamp x y speed heading yaw_rate [status text]
% WAIT and other diagnostic lines in filtered output are skipped and counted.
% Malformed G/O/E data lines cause an error with filename and line number.
%
% Units in files/results: seconds, meters, m/s, radians, rad/s.
% Plot angles/rates and their RMSE in degrees/deg/s for readability.
% Multiple estimates at the same time: use the LAST in file order, i.e. the
% estimate after all updates at that time. All E statuses are included,
% including estimates emitted after gated/rejected measurements.
%
% Truth is interpolated at each measurement/estimate timestamp, never
% extrapolated. Heading is unwrapped before interpolation; errors use the
% shortest angular difference. RMSE compares both methods at identical noisy
% sensor timestamps within truth/estimate coverage. Filter interpolation is
% only for offline comparison; it does not change your filter algorithm.
% For the supplied files the sensor times coincide with filter output times.
% No startup interval is discarded. Missing packets/gaps are not modeled.
%
% Odometry-only position integrates measured speed and measured heading:
%   x = x0 + integral(v*cos(heading) dt)
%   y = y0 + integral(v*sin(heading) dt)
% Ground truth supplies ONLY the initial position at the first odometry time.
% Every displacement thereafter uses noisy O packets only; no GPS or filtered
% state is used. Heading is already supplied, so yaw rate is not integrated.
% This assumes forward speed with negligible sideslip and a common XY frame.
% Near-zero-mean white noise can produce very little visible drift; this plot
% does not by itself establish robustness to persistent odometry biases.
%
% Outputs: three figures, console RMSE table, and a results struct. No input
% files are modified and no figures/files are automatically saved.

if nargin < 1 || isempty(groundFile), groundFile = 'ground_packets.txt'; end
if nargin < 2 || isempty(noisyFile), noisyFile = 'test_packets.txt'; end
if nargin < 3 || isempty(filteredFile), filteredFile = 'test_packets1_filtered.txt'; end
truth = readPackets(groundFile, false);
noisy = readPackets(noisyFile, false);
filt = readPackets(filteredFile, true);
requireStream(truth.G, 'ground-truth GPS');
requireStream(truth.O, 'ground-truth odometry');
requireStream(noisy.G, 'noisy GPS');
requireStream(noisy.O, 'noisy odometry');
requireStream(filt.E, 'filtered estimates');

% Raw measurements use independent GPS and odometry time bases.
rawT = {noisy.G(:,1), noisy.G(:,1), noisy.O(:,1), ...
        noisy.O(:,1), noisy.O(:,1)};
rawY = {noisy.G(:,2), noisy.G(:,3), noisy.O(:,2), ...
        noisy.O(:,3), noisy.O(:,4)};
groundT = {truth.G(:,1), truth.G(:,1), truth.O(:,1), ...
           truth.O(:,1), truth.O(:,1)};
groundY = {truth.G(:,2), truth.G(:,3), truth.O(:,2), ...
           truth.O(:,3), truth.O(:,4)};
tF = filt.E(:,1);
yF = filt.E(:,2:6);

% Dead reckoning using actual packet timestamps (not a fixed sample period).
tO = noisy.O(:,1);
vO = noisy.O(:,2);
headingO = noisy.O(:,3);
initialXY = interpolate(truth.G(:,1), truth.G(:,2:3), tO(1), false);
if any(~isfinite(initialXY))
    error('Ground-truth position must cover the first odometry timestamp.');
end
odomXY = bsxfun(@plus, cumtrapz(tO, ...
    [vO.*cos(headingO), vO.*sin(headingO)]), initialXY);
odomTruthXY = interpolate(truth.G(:,1), truth.G(:,2:3), tO, false);
odomError = odomXY - odomTruthXY;

scale = [1 1 1 180/pi 180/pi];
labels = {'X position (m)', 'Y position (m)', 'Speed (m/s)', ...
          'Heading (deg)', 'Yaw rate (deg/s)'};
rawError = cell(1,5);
filterError = nan(size(yF));
rawRMSE = nan(6,1); filterRMSE = rawRMSE; sampleCount = zeros(6,1);
matchedRaw = cell(1,5); matchedFilter = cell(1,5);

for j = 1:5
    isAngle = (j == 4);
    atRaw = interpolate(groundT{j}, groundY{j}, rawT{j}, isAngle);
    atFilter = interpolate(groundT{j}, groundY{j}, tF, isAngle);
    rawError{j} = difference(rawY{j}, atRaw, isAngle);
    filterError(:,j) = difference(yF(:,j), atFilter, isAngle);
    filterAtRaw = interpolate(tF, yF(:,j), rawT{j}, isAngle);
    er = rawError{j};
    ef = difference(filterAtRaw, atRaw, isAngle);
    valid = isfinite(er) & isfinite(ef);
    matchedRaw{j} = er(valid); matchedFilter{j} = ef(valid);
    sampleCount(j) = sum(valid);
    if sampleCount(j) == 0
        error('No common timestamps for %s.', labels{j});
    end
    rawRMSE(j) = sqrt(mean(er(valid).^2))*scale(j);
    filterRMSE(j) = sqrt(mean(ef(valid).^2))*scale(j);
end
% X and Y share their time coverage, hence these are paired GPS samples.
rawRMSE(6) = sqrt(mean(matchedRaw{1}.^2 + matchedRaw{2}.^2));
filterRMSE(6) = sqrt(mean(matchedFilter{1}.^2 + matchedFilter{2}.^2));
sampleCount(6) = sampleCount(1);
improvement = nan(6,1);
nonzero = rawRMSE > 0;
improvement(nonzero) = 100*(1-filterRMSE(nonzero)./rawRMSE(nonzero));
quantity = {'X'; 'Y'; 'Speed'; 'Heading'; 'Yaw rate'; '2D position'};
unit = {'m'; 'm'; 'm/s'; 'deg'; 'deg/s'; 'm'};
metrics = table(quantity, unit, sampleCount, rawRMSE, filterRMSE, improvement, ...
    'VariableNames', {'Quantity','Unit','Samples','NoisyRMSE', ...
                      'FilteredRMSE','ImprovementPercent'});
fprintf('\nRMSE at matching sensor timestamps; startup included:\n');
disp(metrics);
fprintf('Positive improvement means lower error. 2D RMSE = sqrt(mean(dx^2+dy^2)).\n');
fprintf('Filtered diagnostic lines skipped: %d; duplicate timestamps removed: %d\n', ...
        filt.skipped, filt.duplicates);

% Fair three-way position comparison at common GPS sample timestamps.
tP = noisy.G(:,1);
truthP = interpolate(truth.G(:,1), truth.G(:,2:3), tP, false);
odomP = interpolate(tO, odomXY, tP, false);
filterP = interpolate(tF, yF(:,1:2), tP, false);
validP = all(isfinite([truthP, odomP, filterP]),2);
if ~any(validP)
    error('No common timestamps for the three-way position comparison.');
end
gpsPError = noisy.G(validP,2:3) - truthP(validP,:);
odomPError = odomP(validP,:) - truthP(validP,:);
filterPError = filterP(validP,:) - truthP(validP,:);
positionQuantity = {'X'; 'Y'; '2D position'};
positionSamples = repmat(sum(validP),3,1);
positionMetrics = table(positionQuantity, positionSamples, ...
    positionRMSE(gpsPError), positionRMSE(odomPError), positionRMSE(filterPError), ...
    'VariableNames', {'Quantity','Samples','NoisyGPS_RMSE_m', ...
                     'OdometryOnly_RMSE_m','Filtered_RMSE_m'});
fprintf('\nPosition RMSE at common GPS timestamps (all values in meters):\n');
disp(positionMetrics);
fprintf('Odometry-only starts at truth XY at t=%.6f s; no later position corrections.\n',tO(1));

% Common colors throughout all figures.
cRaw = [0.75 0.45 0.12]; cFilter = [0.05 0.40 0.85];
cOdom = [0.65 0.10 0.65];
figures = gobjects(3,1);
figures(1) = figure('Name','Vehicle path comparison','Color','w');
plot(truth.G(:,2),truth.G(:,3),'k-','LineWidth',1.8); hold on;
plot(noisy.G(:,2),noisy.G(:,3),'.','Color',cRaw,'MarkerSize',8);
plot(yF(:,1),yF(:,2),'-','Color',cFilter,'LineWidth',1.2);
plot(odomXY(:,1),odomXY(:,2),'--','Color',cOdom,'LineWidth',1.4);
axis equal; grid on;
xlabel('X / East (m)'); ylabel('Y / North (m)');
title('Vehicle path');
legend('Ground truth','Noisy GPS','Filtered','Odometry only','Location','best');

figures(2) = figure('Name','State comparison','Color','w');
axesState = gobjects(5,1);
for j = 1:5
    axesState(j) = subplot(3,2,j);
    gt = groundY{j}; nr = rawY{j}; fe = yF(:,j);
    if j == 4
        % Put all heading curves on the same continuous revolution branch.
        gt = unwrap(gt);
        nrRef = interpolate(groundT{j}, gt, rawT{j}, false);
        feRef = interpolate(groundT{j}, gt, tF, false);
        nr = nrRef + difference(nr, nrRef, true);
        fe = feRef + difference(fe, feRef, true);
    end
    plot(groundT{j},gt*scale(j),'k-','LineWidth',1.4); hold on;
    plot(rawT{j},nr*scale(j),'.','Color',cRaw,'MarkerSize',3);
    plot(tF,fe*scale(j),'-','Color',cFilter,'LineWidth',1);
    if j<=2
        plot(tO,odomXY(:,j),'--','Color',cOdom,'LineWidth',1.4);
    end
    grid on; xlabel('Time (s)'); ylabel(labels{j}); title(labels{j});
    if j<=2
        legend('Ground truth','Noisy GPS','Filtered','Odometry only','Location','best');
    elseif j==3
        legend('Ground truth','Noisy odometry','Filtered','Location','best');
    end
end
linkaxes(axesState,'x');
subplot(3,2,6); axis off;
text(0,0.9,{'Heading shown continuously across +/-180 degrees.'; ...
    'Angle errors use the shortest signed difference.'; ...
    'Final estimate retained at each repeated timestamp.'; ...
    'Purple dashed: integral of noisy speed + heading.'; ...
    'Odometry-only starts at truth XY; no later corrections.'; ...
    'RMSE tables printed in the Command Window.'}, ...
    'Units','normalized','VerticalAlignment','top','FontSize',10);

figures(3) = figure('Name','Errors relative to ground truth','Color','w');
axesError = gobjects(6,1);
for j = 1:6
    axesError(j) = subplot(3,2,j);
    if j<=5
        tr = rawT{j}; er = rawError{j}*scale(j); ef = filterError(:,j)*scale(j);
        label = [labels{j} ' error'];
    else
        tr = noisy.G(:,1); er = hypot(rawError{1},rawError{2});
        ef = hypot(filterError(:,1),filterError(:,2)); label = '2D position error (m)';
    end
    plot(tr,er,'.','Color',cRaw,'MarkerSize',3); hold on;
    plot(tF,ef,'-','Color',cFilter,'LineWidth',1);
    if j<=2
        plot(tO,odomError(:,j),'--','Color',cOdom,'LineWidth',1.2);
    elseif j==6
        plot(tO,hypot(odomError(:,1),odomError(:,2)), ...
            '--','Color',cOdom,'LineWidth',1.2);
    end
    limits = xlim;
    plot(limits,[0 0],'k:','HandleVisibility','off');
    grid on; xlabel('Time (s)'); ylabel(label);
    if j<=2 || j==6
        pRow = j; if j==6, pRow=3; end
        title(sprintf('RMSE: GPS %.3g | odom %.3g | filter %.3g', ...
            positionMetrics.NoisyGPS_RMSE_m(pRow), ...
            positionMetrics.OdometryOnly_RMSE_m(pRow), ...
            positionMetrics.Filtered_RMSE_m(pRow)));
        legend('Noisy GPS error','Filtered error','Odometry-only error','Location','best');
    else
        title(sprintf('RMSE: noisy %.4g | filtered %.4g',rawRMSE(j),filterRMSE(j)));
    end
end
linkaxes(axesError,'x');
results = struct('metrics',metrics,'truth',truth,'noisy',noisy, ...
    'filtered',filt,'rawError',{rawError},'filterError',filterError,'figures',figures);
results.positionMetrics = positionMetrics;
results.odometryOnly = struct('time',tO,'xy',odomXY,'error',odomError, ...
    'initialXY',initialXY);
end

function r = positionRMSE(e)
r = [sqrt(mean(e(:,1).^2)); sqrt(mean(e(:,2).^2)); ...
     sqrt(mean(sum(e.^2,2)))];
end

function data = readPackets(path, filtered)
[fid,msg] = fopen(path,'r');
if fid<0, error('Cannot open %s: %s',path,msg); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
lines = textscan(fid,'%s','Delimiter','\n','Whitespace','');
lines = lines{1}; n = numel(lines);
G = zeros(n,3); O = zeros(n,4); E = zeros(n,6);
ng=0; no=0; ne=0; skipped=0;
for k=1:n
    s = strtrim(lines{k});
    if isempty(s) || s(1)=='#' || s(1)=='%', continue; end
    [tag,rest] = strtok(s);
    if filtered && ~strcmp(tag,'E')
        skipped=skipped+1; continue;
    end
    if ~filtered && ~any(strcmp(tag,{'G','O'}))
        error('Unexpected packet type in %s, line %d: %s',path,k,tag);
    end
    if strcmp(tag,'E')
        % Read exactly six numbers; trailing status words are allowed.
        [v,count] = sscanf(rest,'%f',6);
        ok = count==6;
    else
        % G/O must have only numeric fields; reject malformed trailing text.
        [v,count,~,next] = sscanf(rest,'%f');
        ok = isempty(strtrim(rest(next:end)));
        if strcmp(tag,'G'), ok=ok && (count==3 || count==4);
        else, ok=ok && count==4; end
    end
    if ~ok || any(~isfinite(v))
        error('Malformed/nonfinite packet in %s, line %d.',path,k);
    end
    switch tag
        case 'G', ng=ng+1; G(ng,:)=v(1:3)';
        case 'O', no=no+1; O(no,:)=v';
        case 'E', ne=ne+1; E(ne,:)=v';
    end
end
[G,dg] = canonical(G(1:ng,:));
[O,dO] = canonical(O(1:no,:));
[E,de] = canonical(E(1:ne,:));
data = struct('G',G,'O',O,'E',E,'skipped',skipped,'duplicates',dg+dO+de);
end

function [a,removed] = canonical(a)
% unique sorts timestamps and selects the last occurrence in original order.
oldN = size(a,1);
if ~isempty(a)
    [~,idx] = unique(a(:,1),'last');
    a = a(idx,:);
end
removed = oldN-size(a,1);
end

function requireStream(a,name)
if size(a,1)<2, error('Need at least two distinct timestamps for %s.',name); end
end

function yi = interpolate(t,y,tq,isAngle)
if isAngle, y=unwrap(y); end
yi=interp1(t,y,tq,'linear',NaN); % Explicitly prohibit extrapolation.
end

function d = difference(a,b,isAngle)
d=a-b;
if isAngle, d=atan2(sin(d),cos(d)); end
end
