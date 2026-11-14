%% SIMULATE_BUGGY_PACKETS
% Run this script in MATLAB R2016b or newer (base MATLAB only).
% Outputs, beside this script:
%   test_packets.txt   noisy GPS at 5 Hz and odometry at 400 Hz
%   ground_packets.txt noiseless G and O packets at every 1 kHz truth instant
%
% Packet grammar matches kalman_package:
%   G timestamp x y
%   O timestamp signed_forward_speed heading yaw_rate
% Units: seconds, meters, m/s, radians, rad/s. x=East, y=North;
% heading zero is +x; positive rotation is counterclockwise.
%
% This is a reproducible synthetic kinematic course, not a calibrated VN-300
% emulator or an autonomous obstacle planner. The vehicle is already moving
% at the start and remains moving at the finish (a rolling course segment).
% Obstacles are staged beside the curved route and clearance is checked
% against the entire trajectory, including an approximate vehicle footprint.
% No real GNSS fixes, encoder hardware, or sensor manufacturer's guarantees
% are implied. Noise values below are provisional engineering assumptions.

clearvars;
close all;
clc;

%% Configuration
cfg.duration = 60;                  % seconds
cfg.truthHz = 1000;
cfg.odomHz = 400;
cfg.gpsHz = 5;
cfg.averageSpeed = 10 / 3.6;        % 10 km/h
cfg.maxSpeed = 25 / 3.6;            % 25 km/h
cfg.seed = 42;
cfg.makePlots = true;

% Match the C filter's measurement STD constants for the baseline test.
cfg.gpsStd = 1.0;                  % per-axis position STD, meters
cfg.velocityStd = 0.05;            % m/s
cfg.headingStd = 0.00873;          % rad (~0.5 degrees)
cfg.yawRateStd = 0.002;            % rad/s

% Optional colored-noise stress test. false gives independent Gaussian noise
% matching the filter's basic measurement-noise assumption. true preserves
% each marginal STD but correlates successive samples (harder for the filter).
% This does not model shared errors between GPS and INS-derived odometry.
cfg.correlatedNoise = false;
cfg.gpsCorrelationTime = 1.0;       % seconds; used only when enabled
cfg.odomCorrelationTime = 0.10;

% Illustrative small-buggy geometry/limits, not measured vehicle parameters.
cfg.wheelbase = 1.6;               % meters
cfg.maxSteer = 35 * pi / 180;       % radians
cfg.maxLateralAcceleration = 3.0;   % m/s^2
cfg.maxLongitudinalAcceleration = 3.0;
cfg.vehicleRadius = 0.95;          % enclosing disk approximation, meters
cfg.safetyMargin = 0.35;            % extra clearance beyond vehicle disk

scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder), scriptFolder = pwd; end
rng(cfg.seed, 'twister');

%% Ground-truth speed and heading profiles
% Integer-index time grids avoid accumulated timestamp drift.
t = (0:round(cfg.duration * cfg.truthHz))' / cfg.truthHz;

% Each row: [start_seconds end_seconds total_heading_change_degrees].
% First pair is a slalom. Later intervals are larger course turns.
turns = [4 8 25; 8 12 -25; 26 34 70; 39 47 -85; 51 57 45];
assert(all(turns(:,2) <= cfg.duration), 'Duration must include all turns.');
yawRate = zeros(size(t));
turnEnvelope = zeros(size(t));
for k = 1:size(turns,1)
    pulse = raisedCosine(t, turns(k,1), turns(k,2));
    % Normalize numerically so each turn has exactly the specified angle.
    yawRate = yawRate + pulse * (turns(k,3)*pi/180) / trapz(t,pulse);
    turnEnvelope = max(turnEnvelope, pulse);
end
headingUnwrapped = cumtrapz(t, yawRate);

% Slow down during turns; accelerate to 25 km/h on a straight at t=18 s.
slow = 1 - 0.45 * turnEnvelope;
burst = raisedCosine(t, 15, 21);
% Solve the cruise level so integral(speed)/duration is exactly 10 km/h.
meanSlow = trapz(t, slow) / cfg.duration;
meanBurst = trapz(t, burst) / cfg.duration;
cruiseSpeed = (cfg.averageSpeed - cfg.maxSpeed*meanBurst) / ...
             (meanSlow - meanBurst);
speed = cruiseSpeed * slow + (cfg.maxSpeed-cruiseSpeed) * burst;
assert(cruiseSpeed > 0 && cruiseSpeed < cfg.maxSpeed, 'Invalid speed profile.');

% Integrate world-frame velocity independently of the C motion model.
x = cumtrapz(t, speed .* cos(headingUnwrapped));
y = cumtrapz(t, speed .* sin(headingUnwrapped));
heading = wrapAngle(headingUnwrapped);
acceleration = gradient(speed, 1/cfg.truthHz);
lateralAcceleration = speed .* yawRate;
steering = atan(cfg.wheelbase * yawRate ./ speed);
assert(max(speed) <= cfg.maxSpeed + 1e-10, 'Maximum speed exceeded.');
assert(abs(trapz(t,speed)/cfg.duration-cfg.averageSpeed) < 1e-10);
assert(max(abs(steering)) <= cfg.maxSteer, 'Steering limit exceeded.');
assert(max(abs(lateralAcceleration)) <= cfg.maxLateralAcceleration, ...
       'Lateral acceleration limit exceeded.');
assert(max(abs(acceleration)) <= cfg.maxLongitudinalAcceleration, ...
       'Longitudinal acceleration limit exceeded.');

%% Stage circular obstacles near the slalom and course bends
obstacleTimes = [6 10 30 43 54];
obstacleSides = [1 -1 1 -1 1];
obstacles = zeros(numel(obstacleTimes),3);  % [center_x center_y radius]
clearances = zeros(numel(obstacleTimes),1);
for k = 1:numel(obstacleTimes)
    idx = round(obstacleTimes(k)*cfg.truthHz)+1;
    normal = obstacleSides(k)*[-sin(headingUnwrapped(idx)), ...
                               cos(headingUnwrapped(idx))];
    offset = 2.6;
    found = false;
    for attempt = 1:100
        center = [x(idx),y(idx)] + offset*normal;
        radius = 0.75;
        clearance = min(hypot(x-center(1), y-center(2))) ...
                    - radius - cfg.vehicleRadius;
        if clearance >= cfg.safetyMargin
            found = true;
            break;
        end
        offset = offset + 0.25;
    end
    assert(found, 'Unable to place obstacle with required clearance.');
    obstacles(k,:) = [center,radius];
    clearances(k) = clearance;
end

%% Sample truth at sensor times
% 400 Hz means 2.5 ms intervals: half of these timestamps fall between the
% 1 ms truth samples. Interpolate unwrapped heading before wrapping it.
tOdom = (0:round(cfg.duration*cfg.odomHz))' / cfg.odomHz;
tGps = (0:round(cfg.duration*cfg.gpsHz))' / cfg.gpsHz;
trueOdom = interp1(t, [speed,headingUnwrapped,yawRate], tOdom, 'linear');
trueGps = interp1(t, [x,y], tGps, 'linear');

tauGps = 0; tauOdom = 0;
if cfg.correlatedNoise
    tauGps = cfg.gpsCorrelationTime;
    tauOdom = cfg.odomCorrelationTime;
end
noisyGps = trueGps + sensorNoise(numel(tGps), cfg.gpsHz, ...
                                [cfg.gpsStd,cfg.gpsStd], tauGps);
noisyOdom = trueOdom + sensorNoise(numel(tOdom), cfg.odomHz, ...
               [cfg.velocityStd,cfg.headingStd,cfg.yawRateStd], tauOdom);
noisyOdom(:,2) = wrapAngle(noisyOdom(:,2));

%% Export packets, GPS first on equal timestamps
% Both streams include t=0 and t=60. There are no duplicated GPS fixes.
writePackets(fullfile(scriptFolder,'test_packets.txt'), ...
             tGps,noisyGps,tOdom,noisyOdom);
writePackets(fullfile(scriptFolder,'ground_packets.txt'), ...
             t,[x,y],t,[speed,heading,yawRate]);

fprintf('Duration: %.1f s; distance: %.3f m\n',cfg.duration,trapz(t,speed));
fprintf('Mean / peak speed: %.3f / %.3f km/h\n', ...
        3.6*trapz(t,speed)/cfg.duration,3.6*max(speed));
fprintf('Peak longitudinal / lateral acceleration: %.3f / %.3f m/s^2\n', ...
        max(abs(acceleration)),max(abs(lateralAcceleration)));
fprintf('Peak steering angle: %.2f degrees\n',max(abs(steering))*180/pi);
fprintf('Minimum obstacle clearance beyond vehicle disk: %.3f m\n',min(clearances));
fprintf('Test: %d GPS + %d odometry packets\n',numel(tGps),numel(tOdom));
fprintf('Ground: %d paired timestamps (%d packet lines)\n',numel(t),2*numel(t));
fprintf('Files written in: %s\n',scriptFolder);

%% Visual inspection (no extra files are written)
if cfg.makePlots
    figure('Name','Synthetic buggy course','Color','w');
    subplot(2,2,[1 3]); hold on; axis equal; grid on;
    hTruth=plot(x,y,'b-','LineWidth',1.5);
    hGps=plot(noisyGps(:,1),noisyGps(:,2),'.','Color',[0.7 0.7 0.7]);
    a=linspace(0,2*pi,100);
    for k=1:size(obstacles,1)
        plot(obstacles(k,1)+obstacles(k,3)*cos(a), ...
             obstacles(k,2)+obstacles(k,3)*sin(a),'r-','HandleVisibility','off');
        inflated=obstacles(k,3)+cfg.vehicleRadius+cfg.safetyMargin;
        plot(obstacles(k,1)+inflated*cos(a), ...
             obstacles(k,2)+inflated*sin(a),'r:','HandleVisibility','off');
    end
    plot(x(1),y(1),'go','HandleVisibility','off');
    plot(x(end),y(end),'ks','HandleVisibility','off');
    xlabel('East x (m)'); ylabel('North y (m)');
    title('Red: obstacles; dotted: vehicle + safety envelope');
    legend([hTruth,hGps],{'Ground truth','Noisy GPS'},'Location','best');
    subplot(2,2,2); plot(t,3.6*speed); grid on;
    xlabel('Time (s)'); ylabel('Speed (km/h)'); title('Ground-truth speed');
    subplot(2,2,4); plot(t,yawRate*180/pi); grid on;
    xlabel('Time (s)'); ylabel('Yaw rate (deg/s)'); title('Ground-truth turning');
end

%% Local helper functions
function p = raisedCosine(t,startTime,endTime)
    p=zeros(size(t));
    use=t>=startTime & t<=endTime;
    phase=(t(use)-startTime)/(endTime-startTime);
    p(use)=0.5*(1-cos(2*pi*phase));
end

function a = wrapAngle(a)
    a=mod(a+pi,2*pi)-pi;
end

function noise = sensorNoise(n,hz,stds,tau)
    noise=bsxfun(@times,randn(n,numel(stds)),stds);
    if tau>0
        rho=exp(-1/(hz*tau));
        innovationScale=sqrt(1-rho*rho);
        for k=2:n
            noise(k,:)=rho*noise(k-1,:)+innovationScale*noise(k,:);
        end
    end
end

function writePackets(path,tGps,gps,tOdom,odom)
    [fid,msg]=fopen(path,'w');
    if fid<0, error('Cannot open %s: %s',path,msg); end
    cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
    % Nine decimal places preserve both sensor time grids in the text file.
    events=[tGps,zeros(numel(tGps),1),(1:numel(tGps))'; ...
            tOdom,ones(numel(tOdom),1),(1:numel(tOdom))'];
    events=sortrows(events,[1 2]);
    for k=1:size(events,1)
        i=events(k,3);
        if events(k,2)==0
            fprintf(fid,'G %.9f %.12f %.12f\n',events(k,1),gps(i,1),gps(i,2));
        else
            fprintf(fid,'O %.9f %.12f %.12f %.12f\n', ...
                    events(k,1),odom(i,1),odom(i,2),odom(i,3));
        end
    end
    [msg,err]=ferror(fid);
    if err~=0, error('Writing %s failed: %s',path,msg); end
end
