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
%
% Odometry represents a GNSS-aided VN-300's speed, heading and yaw-rate
% outputs, already transformed to the vehicle/ENU frame. It is NOT a raw
% accelerometer/gyro navigation mechanization. Slowly varying output errors
% and persistent offsets cause drift when YOU integrate the O packets.
% GNSS aiding bounds heading/velocity errors; a GNSS outage needs a separate
% model. This model does not reproduce VectorNav's proprietary estimator.
% The VN-300 does not contain dead-wheel encoders. Any external encoders,
% wheel-radius error, slip, sideslip, timing error and lever-arm effects would
% need to be modeled separately if present in your odometry pipeline.
% GPS generation is unchanged. Shared errors between raw GNSS and VN-300
% INS output are not modeled, even if both originate from the same unit.
%
% Magnitude references: supplied VN-300 datasheet (8wfz09s (1).pdf, R2),
% INS Performance and IMU Specifications; public product specifications:
% https://www.vectornav.com/products/detail/vn-300
% Velocity accuracy: 0.05 m/s RMS in supplied datasheet; dynamic heading:
% 0.2 deg 1 sigma with sufficient motion; mounting error: 0.15 deg 1 sigma;
% gyro bias stability: 5 deg/hour typical, density: 0.0035 deg/s/sqrt(Hz).
% These do NOT determine a unique stochastic model. The fast/slow split,
% time constants and fixed test offsets below are engineering assumptions.
% Gyro Allan bias stability is not directly a Gauss-Markov standard deviation.

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

% GPS noise unchanged from the original simulator.
cfg.gpsStd = 1.0;                  % per-axis position STD, meters
cfg.correlatedNoise = false;       % optional correlation of GPS ONLY
cfg.gpsCorrelationTime = 1.0;       % seconds; used only when enabled

% VN-300-like aided output errors. These are NOT manufacturer covariance
% values. Do not set the C filter's R from the fast-noise STD alone: persistent
% errors and temporal correlation also matter. GNSS compass accuracy depends
% on antenna separation/environment; this is a nominal aligned INS scenario.
cfg.odomDriftEnabled = true;       % false: fast noise only for comparison
cfg.velocityStd = 0.020;           % fast output noise STD, m/s
cfg.velocityBias = 0.030;          % fixed +3 cm/s offset for this test run
cfg.velocitySlowStd = 0.025;       % stationary slow error STD, m/s
cfg.velocityCorrelationTime = 20; % seconds
% Combined speed RMS about truth ~sqrt(.020^2+.030^2+.025^2)=.044 m/s.
cfg.headingStd = 0.05*pi/180;      % fast output noise STD, radians
cfg.headingMountingOffset = 0.15*pi/180; % fixed residual installation offset
cfg.headingSlowStd = 0.18*pi/180;  % stationary aided heading error STD
cfg.headingCorrelationTime = 20; % seconds; approximate aiding error memory
cfg.headingAidingTime = 5;         % seconds; limits integrated gyro bias
cfg.gyroBias = (2/3600)*pi/180;    % fixed residual 2 deg/hour, in rad/s
cfg.gyroSlowStd = (3/3600)*pi/180; % assumed slow residual STD, rad/s
cfg.gyroCorrelationTime = 100;     % seconds; NOT derived from Allan minimum
cfg.gyroNoiseDensity = 0.0035*pi/180; % rad/s/sqrt(Hz), specification scale
% Ideal anti-aliasing approximation: noise-equivalent bandwidth fs/2.
% Actual output STD depends on configured sensor filtering/bandwidth.
cfg.yawRateStd = cfg.gyroNoiseDensity*sqrt(cfg.odomHz/2);

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

tauGps = 0;
if cfg.correlatedNoise
    tauGps = cfg.gpsCorrelationTime;
end
noisyGps = trueGps + sensorNoise(numel(tGps), cfg.gpsHz, ...
                                [cfg.gpsStd,cfg.gpsStd], tauGps);

% Fast noise plus errors that remain correlated across many 400 Hz packets.
% sensorNoise(...,tau) uses the exact first-order Gauss-Markov recurrence:
%   b(k)=rho*b(k-1)+sigma*sqrt(1-rho^2)*randn, rho=exp(-dt/tau).
% Its stationary STD stays sigma when the sample rate changes.
nOdom = numel(tOdom);
fastError = sensorNoise(nOdom, cfg.odomHz, ...
    [cfg.velocityStd,cfg.headingStd,cfg.yawRateStd], 0);
slowError = zeros(nOdom,3);
gyroHeadingError = zeros(nOdom,1);
if cfg.odomDriftEnabled
    slowError(:,1) = cfg.velocityBias + sensorNoise(nOdom, cfg.odomHz, ...
        cfg.velocitySlowStd, cfg.velocityCorrelationTime);
    slowError(:,3) = cfg.gyroBias + sensorNoise(nOdom, cfg.odomHz, ...
        cfg.gyroSlowStd, cfg.gyroCorrelationTime);
    % Leaky integration approximates aiding removing gyro-induced heading
    % error: de/dt = gyro_bias - e/T. Do not integrate bias unboundedly while
    % assuming the VN-300 still has valid GNSS/heading aiding.
    rhoHeading = exp(-1/(cfg.odomHz*cfg.headingAidingTime));
    gyroHeadingError(1) = cfg.headingAidingTime*slowError(1,3);
    for k=2:nOdom
        gyroHeadingError(k) = rhoHeading*gyroHeadingError(k-1) + ...
            cfg.headingAidingTime*(1-rhoHeading)*slowError(k-1,3);
    end
    slowError(:,2) = cfg.headingMountingOffset + gyroHeadingError + ...
        sensorNoise(nOdom, cfg.odomHz, cfg.headingSlowStd, ...
                    cfg.headingCorrelationTime);
end
odomError = fastError + slowError;
noisyOdom = trueOdom + odomError;
noisyOdom(:,2) = wrapAngle(noisyOdom(:,2));
assert(all(isfinite(noisyOdom(:))), 'Nonfinite odometry measurement.');

% Show the positional consequence without using GPS to correct integration.
% Truth supplies only the initial position, matching compare_packets.m.
odomXY = bsxfun(@plus, cumtrapz(tOdom, ...
    [noisyOdom(:,1).*cos(noisyOdom(:,2)), ...
     noisyOdom(:,1).*sin(noisyOdom(:,2))]), [x(1),y(1)]);
truthXYatOdom = interp1(t,[x,y],tOdom,'linear');
positionDrift = odomXY - truthXYatOdom;

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
fprintf('Odometry-only endpoint error: %.3f m; 2D RMSE: %.3f m\n', ...
    hypot(positionDrift(end,1),positionDrift(end,2)), ...
    sqrt(mean(sum(positionDrift.^2,2))));
fprintf('Actual output RMS error: speed %.4f m/s, heading %.4f deg, rate %.4f deg/s\n', ...
    sqrt(mean(odomError(:,1).^2)), sqrt(mean(odomError(:,2).^2))*180/pi, ...
    sqrt(mean(odomError(:,3).^2))*180/pi);
fprintf('Slow-error parameters are assumed; this is not calibrated VN-300 data.\n');

%% Visual inspection (no extra files are written)
if cfg.makePlots
    figure('Name','Synthetic buggy course','Color','w');
    subplot(2,2,[1 3]); hold on; axis equal; grid on;
    hTruth=plot(x,y,'b-','LineWidth',1.5);
    hGps=plot(noisyGps(:,1),noisyGps(:,2),'.','Color',[0.7 0.7 0.7]);
    hOdom=plot(odomXY(:,1),odomXY(:,2),'--','Color',[0.65 0.10 0.65], ...
               'LineWidth',1.4);
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
    legend([hTruth,hGps,hOdom],{'Ground truth','Noisy GPS','Odometry only'}, ...
           'Location','best');
    subplot(2,2,2); plot(t,3.6*speed); grid on;
    xlabel('Time (s)'); ylabel('Speed (km/h)'); title('Ground-truth speed');
    subplot(2,2,4); plot(t,yawRate*180/pi); grid on;
    xlabel('Time (s)'); ylabel('Yaw rate (deg/s)'); title('Ground-truth turning');

    figure('Name','Odometry error and accumulated position drift','Color','w');
    errorLabels = {'Speed error (m/s)','Heading error (deg)','Yaw-rate error (deg/s)'};
    errorScale = [1,180/pi,180/pi];
    for j=1:3
        subplot(2,2,j);
        plot(tOdom,odomError(:,j)*errorScale(j),'Color',[0.75 0.75 0.75]); hold on;
        plot(tOdom,slowError(:,j)*errorScale(j),'b-','LineWidth',1.2);
        grid on; xlabel('Time (s)'); ylabel(errorLabels{j});
        if j==1, legend('Total error','Persistent + slow error','Location','best'); end
    end
    subplot(2,2,4);
    plot(tOdom,hypot(positionDrift(:,1),positionDrift(:,2)), ...
         'Color',[0.65 0.10 0.65],'LineWidth',1.3);
    grid on; xlabel('Time (s)'); ylabel('2D position error (m)');
    title('Odometry-only integration, true starting position');
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
