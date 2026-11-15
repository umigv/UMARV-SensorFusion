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
% Sensor models and defaults ported from umigv/maverick, commit:
% 4294faa4d08158548515377130c5ffb0ab6c252a
% src/simulation/sensor_simulator/sensor_simulator/{sensor_simulator.py,
% sensor_simulator_config.py}
% https://github.com/umigv/maverick/tree/4294faa4d08158548515377130c5ffb0ab6c252a/src/simulation/sensor_simulator
%
% Selected input scheme:
%   O velocity and yaw_rate: external dead-wheel encoder model (enc_vel)
%   O heading: VN-300 model (imu orientation, transformed to vehicle frame)
%   G x,y: VN-300 position model (gps, expressed directly in local ENU meters)
% The Python gps topic is fused INS position, NOT raw GNSS antenna output.
% No duplicate VN-300 odom position or INS velocity is fed to the C filter.
% Heading is truth + its own errors, NOT integral(noisy encoder yaw_rate).
%
% Model equations:
%   encoder v = true_v*(1 + OU_scale_v) + white_noise_v
%   encoder w = true_w*(1 + OU_scale_w) + white_noise_w
%   VN heading = true_heading + OU_heading + white_noise_heading
%   VN position = true_position + OU_position + white_noise_position
% All five OU states start at zero; all innovations are independent, as in
% Python. Each state is updated once before the first reported measurement.
% No fixed speed bias, mounting offset, or gyro-to-heading leakage is added.
% The RNG is reproducible within MATLAB, not identical to Python's RNG.
%
% The existing 60-second course/1 kHz truth generator is retained. Sensor
% equations match the repository, not its command subscription/ROS plumbing.
% The reference now uses exact constant-twist arcs between commands; this
% script retains 1 kHz trapezoidal integration of the smooth course profiles.
% Obstacles are staged beside that course; this is not an avoidance planner.
% Speed/heading are assumed to be at a common vehicle reference point with
% mounting transforms already applied. Slip, wheel counts, latency, GNSS
% outages, and cross-quantity INS error coupling are not modeled in this port.
% Repository defaults are simulation parameters, not measured sensor specs.

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

% Keep the previously requested sensor rates by default. The Python node's
% default update_period_s=0.01 publishes ALL sensors at 100 Hz.
cfg.matchPythonPublishRate = false;
if cfg.matchPythonPublishRate
    cfg.odomHz = 100;
    cfg.gpsHz = 100;
end
cfg.sensorHz = cfg.odomHz;          % common hidden-state update rate
assert(cfg.gpsHz <= cfg.sensorHz && ...
    abs(cfg.sensorHz/cfg.gpsHz-round(cfg.sensorHz/cfg.gpsHz)) < 1e-10, ...
    'GPS rate must divide the sensor update rate exactly.');

% EncVelConfig defaults (drift STDs are dimensionless scale factors).
cfg.enc_vel.vx_noise_std_mps = 0.01;
cfg.enc_vel.wz_noise_std_radps = 0.01;
cfg.enc_vel.vx_drift_std = 0.02;    % 2% steady-state scale STD
cfg.enc_vel.wz_drift_std = 0.02;    % 2% steady-state scale STD
cfg.enc_vel.drift_time_constant_s = 60.0;

% Selected Vn300Config defaults. Unused VN velocity/rate outputs are omitted.
cfg.vn300.position_noise_std_m = 0.05;
cfg.vn300.position_drift_std_m = 0.5;
cfg.vn300.position_drift_time_constant_s = 120.0;
cfg.vn300.yaw_noise_std_rad = 0.5*pi/180;
cfg.vn300.yaw_drift_std_rad = 0.3*pi/180;
cfg.vn300.yaw_drift_time_constant_s = 300.0;

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
% At the default 400 Hz, half the 2.5 ms timestamps fall between the 1 ms
% truth samples. Interpolate unwrapped heading before wrapping it.
tOdom = (0:round(cfg.duration*cfg.odomHz))' / cfg.odomHz;
tGps = (0:round(cfg.duration*cfg.gpsHz))' / cfg.gpsHz;
trueOdom = interp1(t, [speed,headingUnwrapped,yawRate], tOdom, 'linear');
trueGps = interp1(t, [x,y], tGps, 'linear');

enc = cfg.enc_vel;
vn = cfg.vn300;
nOdom = numel(tOdom);
% All selected sensor errors evolve on the same internal update grid, like
% the Python update(). GPS exports every sensorHz/gpsHz-th measurement.
% OU initial state is ZERO, rather than a sample from its stationary law.
encScaleV = ouSeries(nOdom,cfg.sensorHz,enc.vx_drift_std,enc.drift_time_constant_s);
encScaleW = ouSeries(nOdom,cfg.sensorHz,enc.wz_drift_std,enc.drift_time_constant_s);
vnHeadingDrift = ouSeries(nOdom,cfg.sensorHz,vn.yaw_drift_std_rad, ...
                          vn.yaw_drift_time_constant_s);
vnPositionDrift = [ouSeries(nOdom,cfg.sensorHz,vn.position_drift_std_m, ...
                            vn.position_drift_time_constant_s), ...
                   ouSeries(nOdom,cfg.sensorHz,vn.position_drift_std_m, ...
                            vn.position_drift_time_constant_s)];

% Assemble the mixed-source O packet in the established order [v,theta,w].
% Multiplicative encoder error vanishes at zero true speed/rate; additive
% encoder white noise remains. Absolute heading has independent VN errors.
slowError = [trueOdom(:,1).*encScaleV, vnHeadingDrift, ...
             trueOdom(:,3).*encScaleW];
fastError = [enc.vx_noise_std_mps*randn(nOdom,1), ...
             vn.yaw_noise_std_rad*randn(nOdom,1), ...
             enc.wz_noise_std_radps*randn(nOdom,1)];
odomError = slowError + fastError;
noisyOdom = trueOdom + odomError;
noisyOdom(:,2) = wrapAngle(noisyOdom(:,2));

% Python's gps is the noisy VN position converted to lat/lon. Our G packets
% keep the same underlying position in ENU meters (no coordinate round trip).
truePositionAtSensor = interp1(t,[x,y],tOdom,'linear');
vnPosition = truePositionAtSensor + vnPositionDrift + ...
             vn.position_noise_std_m*randn(nOdom,2);
gpsStep = round(cfg.sensorHz/cfg.gpsHz);
gpsIndices = (1:gpsStep:nOdom)';
assert(numel(gpsIndices)==numel(tGps) && ...
       max(abs(tOdom(gpsIndices)-tGps))<1e-9, 'Sensor grids do not align.');
noisyGps = vnPosition(gpsIndices,:);
gpsError = noisyGps-trueGps;
assert(all(isfinite(noisyOdom(:))) && all(isfinite(noisyGps(:))), ...
       'Nonfinite sensor measurement.');

% Python-style reported marginal covariances, kept in the MATLAB workspace.
% The packet format has no covariance fields. Your C R must be configured
% separately. These include stationary drift variance but do not represent
% correlations across time. Since OU starts at zero, early actual variance
% is smaller than stationary variance: sigma^2*(1-exp(-2*(k+1)*dt/tau)).
measurementVariance.odom = [ ...
    enc.vx_noise_std_mps^2 + (noisyOdom(:,1)*enc.vx_drift_std).^2, ...
    repmat(vn.yaw_noise_std_rad^2+vn.yaw_drift_std_rad^2,nOdom,1), ...
    enc.wz_noise_std_radps^2 + (noisyOdom(:,3)*enc.wz_drift_std).^2];
measurementVariance.gps = repmat( ...
    vn.position_noise_std_m^2+vn.position_drift_std_m^2,numel(tGps),2);
% Column orders: odom=[var(v),var(theta),var(w)], GPS=[var(x),var(y)].

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
fprintf('G: VN fused position; O: encoder v, VN heading, encoder angular velocity.\n');
fprintf('Rates: truth %g Hz, O %g Hz, G %g Hz.\n',cfg.truthHz,cfg.odomHz,cfg.gpsHz);
fprintf('Python-style marginal GPS STD %.4f m/axis; heading STD %.4f rad.\n', ...
    sqrt(measurementVariance.gps(1,1)),sqrt(measurementVariance.odom(1,2)));
fprintf('Encoder variances depend on speed/rate; see measurementVariance in workspace.\n');
fprintf('Repository defaults reproduced; these are not calibrated hardware errors.\n');

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

    figure('Name','Mixed-source odometry error and position drift','Color','w');
    errorLabels = {'Speed error (m/s)','Heading error (deg)','Yaw-rate error (deg/s)'};
    errorScale = [1,180/pi,180/pi];
    for j=1:3
        subplot(2,2,j);
        plot(tOdom,odomError(:,j)*errorScale(j),'Color',[0.75 0.75 0.75]); hold on;
        plot(tOdom,slowError(:,j)*errorScale(j),'b-','LineWidth',1.2);
        grid on; xlabel('Time (s)'); ylabel(errorLabels{j});
        if j==1, legend('Total error','Slow error','Location','best'); end
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

function drift = ouSeries(n,hz,sigma,tau)
    % Same recurrence/zero initialization as SensorSimulator.ou_update().
    assert(isfinite(hz) && hz>0 && isfinite(sigma) && sigma>=0 && ...
           isfinite(tau) && tau>0, 'Invalid OU parameters.');
    dt = 1/hz;
    decay = exp(-dt/tau);
    innovationStd = sigma*sqrt(max(0,1-decay^2));
    drift = zeros(n,1);
    value = 0;
    for k=1:n
        value = value*decay + innovationStd*randn;
        drift(k) = value;
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
