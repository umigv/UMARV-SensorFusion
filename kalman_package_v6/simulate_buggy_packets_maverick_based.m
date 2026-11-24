%% SIMULATE_BUGGY_PACKETS_MAVERICK_BASED
% Base MATLAB R2016b+ script. Outputs beside this script:
%   ground_packets.txt: noiseless VN AND E packets at 1600 Hz each.
%   test_packets.txt:   noisy VN and E at the independently configurable rates below.
%
% Packets: VN timestamp x y heading angular_velocity
%          E  timestamp signed_forward_velocity
% Units: seconds, meters, radians, rad/s, m/s. Local ENU: x east, y north;
% heading zero is east, positive rotation counterclockwise.
% Compatible with the five-state SRUKF: [x, y, heading, angular_velocity, v].
% E precedes VN at equal timestamps for deterministic ordering; the filter
% accepts either order. Both streams start at zero, providing its initial pair.
%
% Sensor output-error equations/defaults match umigv/maverick commit
% 4294faa4d08158548515377130c5ffb0ab6c252a, sensor_simulator.py and config:
% https://github.com/umigv/maverick/tree/4294faa4d08158548515377130c5ffb0ab6c252a/src/simulation/sensor_simulator
%   E v       = true_v*(1 + OU_scale) + white_noise
%   VN x,y    = true_position + OU_position + white_noise
%   VN theta  = true_heading + OU_heading + white_noise
%   VN omega  = true_omega + OU_angular_rate + white_noise
% VN quantities approximate the already-filtered INS outputs. No raw IMU or
% GNSS filter is simulated. The external encoder angular-rate output from the
% reference is intentionally NOT used: this vehicle's omega comes from VN.
% Heading and position have their own output errors; neither is generated
% by integrating noisy omega or encoder speed. Five independent OU states
% start at zero and update before the first grid sample, as in the reference.
%
% The Python node publishes everything at 100 Hz by default. Here the requested
% output rates are configurable. Noise is generated on the fixed truthHz grid,
% with a separate random substream for each drift and white-noise component.
% Keep seed, truthHz and duration fixed when comparing sensor output rates:
% changing one sensor's rate cannot change the other sensor's packets, and
% shared sample times use the same errors. OU uses the reference recurrence
% at 1/truthHz; off-grid drift is linearly interpolated (an approximation).
% White noise uses the nearest grid sample, never interpolation or averaging;
% sensor rates <= truthHz ensure distinct samples within each sensor stream.
% Per-sample white-noise STDs stay at the repository values (no sqrt(rate)
% scaling). Random sequences are repeatable in MATLAB, not identical to Python.
% This changes the old random realization, not the configured noise levels.
%
% The original 60-second course and obstacles are retained; truth integration
% now uses 1600 Hz. Sensor error equations match the repository; its ROS command
% subscription, fixed timer, and constant-twist ground-truth integration are
% not copied. Our smooth speed/turn profiles use trapezoidal truth integration.
% Obstacles are staged beside the path, not handled by an avoidance planner.
% Mounting/reference-point transforms are assumed already applied. Wheel ticks,
% slip, latency, outages, and cross-quantity INS error correlations are omitted,
% as in the selected simplified output model. Defaults are not hardware specs.

clearvars;
close all;
clc;

%% Configuration
cfg.duration = 60;                  % seconds
cfg.truthHz = 1600;               % VN and E ground-truth rate, each
cfg.vnHz = 400;                    % noisy VN packet rate
cfg.encoderHz = 1600;               % noisy E packet rate
cfg.averageSpeed = 10 / 3.6;        % 10 km/h
cfg.maxSpeed = 25 / 3.6;            % 25 km/h
cfg.seed = 42;
cfg.makePlots = true;

% Example alternatives: cfg.vnHz=100; cfg.encoderHz=400.
% No integer rate ratio or coincident sensor times are required after startup.
% The five-state filter predicts to every VN or E timestamp using its stored
% speed and yaw rate, then corrects from that packet's measured quantities.
assert(isscalar(cfg.duration) && isfinite(cfg.duration) && cfg.duration>0);
rates = [cfg.truthHz,cfg.vnHz,cfg.encoderHz];
assert(all(isfinite(rates)) && all(rates>0) && ...
       all(rates(2:3)<=cfg.truthHz), ...
       'Sensor rates must be positive and no greater than truthHz.');
assert(abs(cfg.duration*cfg.truthHz-round(cfg.duration*cfg.truthHz))<1e-8, ...
       'Duration must contain an integer number of truth steps.');

% EncVelConfig: only forward velocity is used for this vehicle.
cfg.enc_vel.vx_noise_std_mps = 0.01;
cfg.enc_vel.vx_drift_std = 0.02;   % dimensionless, 2% stationary scale STD
cfg.enc_vel.drift_time_constant_s = 60.0;

% Vn300Config: fused position, absolute heading, and compensated angular rate.
cfg.vn300.position_noise_std_m = 0.05;
cfg.vn300.position_drift_std_m = 0.5;
cfg.vn300.position_drift_time_constant_s = 120.0;
cfg.vn300.yaw_noise_std_rad = 0.5*pi/180;
cfg.vn300.yaw_drift_std_rad = 0.3*pi/180;
cfg.vn300.yaw_drift_time_constant_s = 300.0;
cfg.vn300.wz_noise_std_radps = 0.005;
cfg.vn300.wz_drift_std_radps = 0.002;
cfg.vn300.vel_drift_time_constant_s = 120.0;

% Illustrative small-buggy geometry/limits, not measured vehicle parameters.
cfg.wheelbase = 1.6;               % meters
cfg.maxSteer = 35 * pi / 180;       % radians
cfg.maxLateralAcceleration = 3.0;   % m/s^2
cfg.maxLongitudinalAcceleration = 3.0;
cfg.vehicleRadius = 0.95;          % enclosing disk approximation, meters
cfg.safetyMargin = 0.35;            % extra clearance beyond vehicle disk

scriptFolder = fileparts(mfilename('fullpath'));
if isempty(scriptFolder), scriptFolder = pwd; end
assert(isscalar(cfg.seed) && isfinite(cfg.seed) && cfg.seed>=0 && ...
       cfg.seed<=2^32-1 && cfg.seed==floor(cfg.seed), 'Invalid random seed.');
% Distinct mrg32k3a substreams avoid dependence on generation order/count.
% 1: encoder scale; 2:5: VN drift; 6: encoder white; 7:10: VN white.
noiseStreams = cell(10,1);
for noiseIndex=1:10
    noiseStreams{noiseIndex}=RandStream('mrg32k3a','Seed',cfg.seed);
    noiseStreams{noiseIndex}.Substream=noiseIndex;
end

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

%% Sample truth independently at each sensor's times
% floor avoids extrapolating when duration*rate is not an integer. All streams
% start at t=0; t=duration is included only if it belongs to that rate's grid.
tVn = sampleTimes(cfg.duration,cfg.vnHz);
tEncoder = sampleTimes(cfg.duration,cfg.encoderHz);
trueVn = interp1(t,[x,y,headingUnwrapped,yawRate],tVn,'linear');
trueEncoder = interp1(t,speed,tEncoder,'linear');
nVn = numel(tVn); nEncoder = numel(tEncoder);
enc = cfg.enc_vel; vn = cfg.vn300;

% Generate a common underlying realization before sampling either sensor.
nNoise = numel(t);
encScaleGrid = ouSeries(nNoise,cfg.truthHz,enc.vx_drift_std, ...
                       enc.drift_time_constant_s,noiseStreams{1});
vnDriftStd = [vn.position_drift_std_m,vn.position_drift_std_m, ...
              vn.yaw_drift_std_rad,vn.wz_drift_std_radps];
vnDriftTau = [vn.position_drift_time_constant_s,vn.position_drift_time_constant_s, ...
              vn.yaw_drift_time_constant_s,vn.vel_drift_time_constant_s];
vnSlowGrid = zeros(nNoise,4);
vnWhiteGrid = zeros(nNoise,4);
for j=1:4
    vnSlowGrid(:,j) = ouSeries(nNoise,cfg.truthHz,vnDriftStd(j), ...
                               vnDriftTau(j),noiseStreams{j+1});
    vnWhiteGrid(:,j) = randn(noiseStreams{j+6},nNoise,1);
end
encoderWhiteGrid = randn(noiseStreams{6},nNoise,1);
encScale = interp1(t,encScaleGrid,tEncoder,'linear');
vnSlowError = interp1(t,vnSlowGrid,tVn,'linear');
% Nearest-grid selection retains white-noise variance at off-grid times.
vnNoiseIndex = round(tVn*cfg.truthHz)+1;
encoderNoiseIndex = round(tEncoder*cfg.truthHz)+1;
assert(all(diff(vnNoiseIndex)>0) && all(diff(encoderNoiseIndex)>0), ...
       'Noise samples must be distinct within each sensor stream.');
vnWhiteStd = [vn.position_noise_std_m,vn.position_noise_std_m, ...
              vn.yaw_noise_std_rad,vn.wz_noise_std_radps];
vnError = vnSlowError + bsxfun(@times,vnWhiteGrid(vnNoiseIndex,:),vnWhiteStd);
noisyVn = trueVn + vnError;
noisyVn(:,3) = wrapAngle(noisyVn(:,3));
encoderSlowError = trueEncoder.*encScale;
encoderError = encoderSlowError + ...
               enc.vx_noise_std_mps*encoderWhiteGrid(encoderNoiseIndex);
noisyEncoder = trueEncoder + encoderError;
assert(all(isfinite(noisyVn(:))) && all(isfinite(noisyEncoder(:))), ...
       'Nonfinite sensor measurement.');

% Reference-reported marginal variances (not exported in the text packets).
% These include stationary OU variance and do not model time correlation.
% OU starts at zero, so early actual drift variance is below stationary variance.
measurementVariance.VN = repmat([ ...
    vn.position_noise_std_m^2+vn.position_drift_std_m^2, ...
    vn.position_noise_std_m^2+vn.position_drift_std_m^2, ...
    vn.yaw_noise_std_rad^2+vn.yaw_drift_std_rad^2, ...
    vn.wz_noise_std_radps^2+vn.wz_drift_std_radps^2],nVn,1);
measurementVariance.E = enc.vx_noise_std_mps^2 + ...
                       (noisyEncoder*enc.vx_drift_std).^2;
% VN columns: var(x), var(y), var(theta), var(omega). E: var(v).

% Diagnostic dead reckoning uses raw E speed and raw VN heading only.
% Heading is interpolated offline on its unwrapped branch, without extrapolation.
% Use the shared time coverage when independently configured grids end apart.
useEncoder = tEncoder>=tVn(1) & tEncoder<=tVn(end);
tDr = tEncoder(useEncoder);
headingAtEncoder = interp1(tVn,unwrap(noisyVn(:,3)),tDr,'linear');
initialXY = interp1(t,[x,y],tDr(1),'linear');
vDr = noisyEncoder(useEncoder);
deadReckoningXY = bsxfun(@plus,cumtrapz(tDr, ...
    [vDr.*cos(headingAtEncoder),vDr.*sin(headingAtEncoder)]),initialXY);
positionDrift = deadReckoningXY-interp1(t,[x,y],tDr,'linear');

%% Export E before VN at equal timestamps
writePackets(fullfile(scriptFolder,'test_packets.txt'), ...
             tVn,noisyVn,tEncoder,noisyEncoder);
writePackets(fullfile(scriptFolder,'ground_packets.txt'), ...
             t,[x,y,heading,yawRate],t,speed);

fprintf('Duration: %.1f s; distance: %.3f m\n',cfg.duration,trapz(t,speed));
fprintf('Mean / peak speed: %.3f / %.3f km/h\n', ...
        3.6*trapz(t,speed)/cfg.duration,3.6*max(speed));
fprintf('Peak longitudinal / lateral acceleration: %.3f / %.3f m/s^2\n', ...
        max(abs(acceleration)),max(abs(lateralAcceleration)));
fprintf('Peak steering angle: %.2f degrees\n',max(abs(steering))*180/pi);
fprintf('Minimum obstacle clearance beyond vehicle disk: %.3f m\n',min(clearances));
fprintf('Test: %d VN + %d E packets\n',nVn,nEncoder);
fprintf('Ground: %d VN + %d E packets (%d lines)\n',numel(t),numel(t),2*numel(t));
fprintf('Rates: truth %g Hz each; VN %g Hz; E %g Hz.\n', ...
        cfg.truthHz,cfg.vnHz,cfg.encoderHz);
fprintf('Files written in: %s\n',scriptFolder);
fprintf('Encoder + VN heading endpoint error: %.3f m; 2D RMSE: %.3f m\n', ...
    hypot(positionDrift(end,1),positionDrift(end,2)), ...
    sqrt(mean(sum(positionDrift.^2,2))));
fprintf('Encoder velocity RMS error %.4f m/s\n',sqrt(mean(encoderError.^2)));
fprintf('VN RMS errors: x %.4f m, y %.4f m, heading %.4f deg, rate %.4f deg/s\n', ...
    sqrt(mean(vnError(:,1).^2)),sqrt(mean(vnError(:,2).^2)), ...
    sqrt(mean(vnError(:,3).^2))*180/pi,sqrt(mean(vnError(:,4).^2))*180/pi);
fprintf('VN omega uses additive rate error; encoder scale error applies only to v.\n');
fprintf('Marginal variances are in measurementVariance; configure filter R separately.\n');

%% Visual inspection (no extra files are written)
if cfg.makePlots
    figure('Name','Synthetic buggy course','Color','w');
    subplot(2,2,[1 3]); hold on; axis equal; grid on;
    hTruth=plot(x,y,'k-','LineWidth',1.5);
    hVn=plot(noisyVn(:,1),noisyVn(:,2),'.','Color',[0.7 0.5 0.2]);
    hDr=plot(deadReckoningXY(:,1),deadReckoningXY(:,2),'--', ...
             'Color',[0.65 0.10 0.65],'LineWidth',1.4);
    a=linspace(0,2*pi,100);
    for k=1:size(obstacles,1)
        plot(obstacles(k,1)+obstacles(k,3)*cos(a), ...
             obstacles(k,2)+obstacles(k,3)*sin(a),'r-','HandleVisibility','off');
        inflated=obstacles(k,3)+cfg.vehicleRadius+cfg.safetyMargin;
        plot(obstacles(k,1)+inflated*cos(a), ...
             obstacles(k,2)+inflated*sin(a),'r:','HandleVisibility','off');
    end
    xlabel('East x (m)'); ylabel('North y (m)');
    title('Red: obstacles; dotted: vehicle + safety envelope');
    legend([hTruth,hVn,hDr], ...
        {'Ground truth','VN position','Encoder + VN heading'},'Location','best');
    subplot(2,2,2); plot(t,3.6*speed); grid on;
    xlabel('Time (s)'); ylabel('Speed (km/h)'); title('Ground-truth speed');
    subplot(2,2,4); plot(t,yawRate*180/pi); grid on;
    xlabel('Time (s)'); ylabel('Yaw rate (deg/s)'); title('Ground-truth turning');

    figure('Name','Sensor output errors','Color','w');
    errorLabels={'VN x error (m)','VN y error (m)', ...
                 'VN heading error (deg)','VN rate error (deg/s)'};
    errorScale=[1,1,180/pi,180/pi];
    for j=1:4
        subplot(3,2,j);
        plot(tVn,vnError(:,j)*errorScale(j),'Color',[0.75 0.75 0.75]); hold on;
        plot(tVn,vnSlowError(:,j)*errorScale(j),'b-','LineWidth',1.2);
        grid on; xlabel('Time (s)'); ylabel(errorLabels{j});
        if j==1, legend('Total error','OU drift','Location','best'); end
    end
    subplot(3,2,5);
    plot(tEncoder,encoderError,'Color',[0.75 0.75 0.75]); hold on;
    plot(tEncoder,encoderSlowError,'b-','LineWidth',1.2);
    grid on; xlabel('Time (s)'); ylabel('Encoder speed error (m/s)');
    subplot(3,2,6);
    plot(tDr,hypot(positionDrift(:,1),positionDrift(:,2)), ...
         'Color',[0.65 0.10 0.65],'LineWidth',1.3);
    grid on; xlabel('Time (s)'); ylabel('2D position error (m)');
    title('Raw speed + VN heading; no position corrections');
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

function drift = ouSeries(n,hz,sigma,tau,stream)
    % Same recurrence/zero initialization as SensorSimulator.ou_update().
    assert(isfinite(hz) && hz>0 && isfinite(sigma) && sigma>=0 && ...
           isfinite(tau) && tau>0, 'Invalid OU parameters.');
    dt = 1/hz;
    decay = exp(-dt/tau);
    innovationStd = sigma*sqrt(-expm1(-2*dt/tau));
    drift = zeros(n,1);
    value = 0;
    for k=1:n
        value = value*decay + innovationStd*randn(stream,1,1);
        drift(k) = value;
    end
end

function times = sampleTimes(duration,hz)
    last=floor(duration*hz+32*eps(max(1,duration*hz)));
    times=(0:last)'/hz;
    if abs(times(end)-duration)<32*eps(max(1,duration)), times(end)=duration; end
    assert(numel(times)>=2,'Each sensor must produce at least two samples.');
end

function writePackets(path,tVn,vn,tEncoder,encoder)
    [fid,msg]=fopen(path,'w');
    if fid<0, error('Cannot open %s: %s',path,msg); end
    cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
    % Sort at the precision written to disk so ties are always E then VN.
    events=[tEncoder,zeros(numel(tEncoder),1),(1:numel(tEncoder))'; ...
            tVn,ones(numel(tVn),1),(1:numel(tVn))'];
    events(:,1)=round(events(:,1)*1e9)/1e9;
    events=sortrows(events,[1 2]);
    for k=1:size(events,1)
        i=events(k,3);
        if events(k,2)==0
            fprintf(fid,'E %.9f %.12f\n',events(k,1),encoder(i));
        else
            fprintf(fid,'VN %.9f %.12f %.12f %.12f %.12f\n', ...
                    events(k,1),vn(i,1),vn(i,2),vn(i,3),vn(i,4));
        end
    end
    [msg,err]=ferror(fid);
    if err~=0, error('Writing %s failed: %s',path,msg); end
end
