function results = compare_packets(groundFile, noisyFile, filteredFile, makePlots)
%COMPARE_PACKETS Compare VN/E packets with the five-state SRUKF navigation output.
% Base MATLAB R2016b+; no toolboxes required.
%   results = compare_packets;
%   results = compare_packets('ground_packets.txt','test_packets.txt', ...
%                             'filtered_packets.txt');
% Pass false as fourth argument to calculate metrics without creating figures.
%
% Input packets (truth and noisy files):
%   VN timestamp x y heading angular_velocity
%   E  timestamp signed_forward_velocity
% Filter output (four published quantities; velocity is internal):
%   F  timestamp x y heading angular_velocity [status text]
% Units: seconds, meters, radians, rad/s, m/s; ENU, heading zero east, CCW.
% Numeric VN/E fields are exact; F permits trailing status words. Malformed
% data lines error with file and line number. Filter diagnostics/WAIT are
% skipped. Old G/O packets or E estimate packets are not accepted.
%
% VN/E rates may differ. Sort each stream by time and keep the LAST occurrence
% at duplicate timestamps. For F this is the estimate after all updates at a
% timestamp. Rejected/failed-update estimates are retained, not cherry-picked.
% Truth and estimates are interpolated only within coverage, never extrapolated.
% Heading interpolation uses unwrapped angles; errors use shortest differences.
% Navigation RMSE compares VN and filter at identical VN timestamps. The filter
% state is [x, y, heading, angular_velocity, velocity], but F does not publish
% velocity. Encoder speed therefore has a raw-versus-truth comparison only.
% Both VN and E packets cause prediction and correction; F status words such
% as vn_updated and encoder_updated are accepted without special handling.
%
% Purple dashed position integrates RAW E speed and RAW VN heading, using
% trapezoidal integration at encoder times. Heading is interpolated offline
% (it can use a future VN sample); this is a diagnostic, not a causal estimator.
% Truth supplies ONLY the starting XY. No noisy VN position or filtered values
% are used for this trajectory. Omega is not integrated: absolute VN heading
% is already supplied. Label: 'Encoder + VN heading', not encoder-only pose.
% Assumes a common reference point, planar motion and negligible sideslip.
% Long gaps are interpolated across; no dropout model or warm-up exclusion.
%
% Outputs: figures, console tables, and results. No files are modified/saved.

if nargin<1 || isempty(groundFile), groundFile='ground_packets.txt'; end
if nargin<2 || isempty(noisyFile), noisyFile='test_packets.txt'; end
if nargin<3 || isempty(filteredFile), filteredFile='filtered_packets.txt'; end
if nargin<4 || isempty(makePlots), makePlots=true; end
truth=readPackets(groundFile,false);
noisy=readPackets(noisyFile,false);
filt=readPackets(filteredFile,true);
requireStream(truth.VN,'ground-truth VN');
requireStream(truth.E,'ground-truth encoder');
requireStream(noisy.VN,'noisy VN');
requireStream(noisy.E,'noisy encoder');
requireStream(filt.F,'filtered estimates');

tVn=noisy.VN(:,1); raw=noisy.VN(:,2:5);
tF=filt.F(:,1); yF=filt.F(:,2:5);
tG=truth.VN(:,1); yG=truth.VN(:,2:5);
scale=[1,1,180/pi,180/pi];
labels={'X position (m)','Y position (m)','Heading (deg)','Yaw rate (deg/s)'};
rawError=nan(size(raw)); filterError=nan(size(yF));
matchedRaw=nan(size(raw)); matchedFilter=nan(size(raw));
rawRMSE=nan(5,1); filterRMSE=nan(5,1); sampleCount=zeros(5,1);
for j=1:4
    isAngle=(j==3);
    truthAtRaw=interpolate(tG,yG(:,j),tVn,isAngle);
    truthAtFilter=interpolate(tG,yG(:,j),tF,isAngle);
    filterAtRaw=interpolate(tF,yF(:,j),tVn,isAngle);
    rawError(:,j)=difference(raw(:,j),truthAtRaw,isAngle);
    filterError(:,j)=difference(yF(:,j),truthAtFilter,isAngle);
    matchedRaw(:,j)=rawError(:,j);
    matchedFilter(:,j)=difference(filterAtRaw,truthAtRaw,isAngle);
    valid=isfinite(matchedRaw(:,j)) & isfinite(matchedFilter(:,j));
    sampleCount(j)=sum(valid);
    if ~any(valid), error('No common timestamps for %s.',labels{j}); end
    rawRMSE(j)=sqrt(mean(matchedRaw(valid,j).^2))*scale(j);
    filterRMSE(j)=sqrt(mean(matchedFilter(valid,j).^2))*scale(j);
end
validXY=all(isfinite([matchedRaw(:,1:2),matchedFilter(:,1:2)]),2);
rawRMSE(5)=sqrt(mean(sum(matchedRaw(validXY,1:2).^2,2)));
filterRMSE(5)=sqrt(mean(sum(matchedFilter(validXY,1:2).^2,2)));
sampleCount(5)=sum(validXY);
improvement=nan(5,1);
nonzero=rawRMSE>0;
improvement(nonzero)=100*(1-filterRMSE(nonzero)./rawRMSE(nonzero));
quantity={'X';'Y';'Heading';'Yaw rate';'2D position'};
unit={'m';'m';'deg';'deg/s';'m'};
metrics=table(quantity,unit,sampleCount,rawRMSE,filterRMSE,improvement, ...
    'VariableNames',{'Quantity','Unit','Samples','VN_RMSE', ...
                     'FilteredRMSE','ImprovementPercent'});
fprintf('\nRMSE at common VN timestamps; startup included:\n'); disp(metrics);
fprintf('Positive improvement means lower error. Angles/rates above use degrees.\n');

% E measures the velocity state, but F does not publish its filtered value.
tE=noisy.E(:,1); vE=noisy.E(:,2);
truthSpeed=interpolate(truth.E(:,1),truth.E(:,2),tE,false);
encoderError=vE-truthSpeed;
validSpeed=isfinite(encoderError);
if ~any(validSpeed), error('No truth coverage for encoder speed.'); end
encoderRMSE=sqrt(mean(encoderError(validSpeed).^2));
encoderMetrics=table(sum(validSpeed),encoderRMSE, ...
    'VariableNames',{'Samples','EncoderSpeed_RMSE_mps'});
fprintf('\nEncoder speed compared with ground truth (filtered speed absent from F output):\n');
disp(encoderMetrics);
fprintf('Filtered diagnostics skipped: %d; duplicate F timestamps removed: %d\n', ...
        filt.skipped,filt.duplicates);

% Baseline using noisy speed and noisy heading, with truth only for initial XY.
useE=tE>=tVn(1) & tE<=tVn(end) & tE>=tG(1) & tE<=tG(end);
tDr=tE(useE); vDr=vE(useE);
if numel(tDr)<2, error('Need two encoder times within VN/truth coverage.'); end
headingDr=interpolate(tVn,raw(:,3),tDr,true);
initialXY=interpolate(tG,yG(:,1:2),tDr(1),false);
drXY=bsxfun(@plus,cumtrapz(tDr, ...
    [vDr.*cos(headingDr),vDr.*sin(headingDr)]),initialXY);
drTruth=interpolate(tG,yG(:,1:2),tDr,false);
drError=drXY-drTruth;

% Three-way position comparison uses the same subset of VN epochs for all.
truthP=interpolate(tG,yG(:,1:2),tVn,false);
drP=interpolate(tDr,drXY,tVn,false);
filterP=interpolate(tF,yF(:,1:2),tVn,false);
validP=all(isfinite([truthP,drP,filterP,raw(:,1:2)]),2);
if ~any(validP), error('No common times for three-way position comparison.'); end
positionQuantity={'X';'Y';'2D position'};
positionSamples=repmat(sum(validP),3,1);
positionMetrics=table(positionQuantity,positionSamples, ...
    positionRMSE(raw(validP,1:2)-truthP(validP,:)), ...
    positionRMSE(drP(validP,:)-truthP(validP,:)), ...
    positionRMSE(filterP(validP,:)-truthP(validP,:)), ...
    'VariableNames',{'Quantity','Samples','VN_RMSE_m', ...
                     'EncoderHeading_RMSE_m','Filtered_RMSE_m'});
fprintf('\nPosition RMSE at common VN timestamps (meters):\n'); disp(positionMetrics);
fprintf('Integrated baseline starts at truth XY at t=%.9f; no later position corrections.\n',tDr(1));

figures=[];
if makePlots
    cRaw=[0.75,0.45,0.12]; cFilter=[0.05,0.40,0.85]; cDr=[0.65,0.10,0.65];
    figures=gobjects(3,1);
    figures(1)=figure('Name','Vehicle path comparison','Color','w');
    plot(yG(:,1),yG(:,2),'k-','LineWidth',1.8); hold on;
    plot(raw(:,1),raw(:,2),'.','Color',cRaw,'MarkerSize',4);
    plot(yF(:,1),yF(:,2),'-','Color',cFilter,'LineWidth',1.2);
    plot(drXY(:,1),drXY(:,2),'--','Color',cDr,'LineWidth',1.4);
    axis equal; grid on; xlabel('X / East (m)'); ylabel('Y / North (m)');
    title('Vehicle path');
    legend('Ground truth','VN position','Filtered','Encoder + VN heading','Location','best');

    figures(2)=figure('Name','State and encoder comparison','Color','w');
    axesState=gobjects(5,1);
    for j=1:4
        axesState(j)=subplot(3,2,j);
        gt=yG(:,j); nr=raw(:,j); fe=yF(:,j);
        if j==3
            gt=unwrap(gt);
            nrRef=interpolate(tG,gt,tVn,false);
            feRef=interpolate(tG,gt,tF,false);
            nr=nrRef+difference(nr,nrRef,true);
            fe=feRef+difference(fe,feRef,true);
        end
        plot(tG,gt*scale(j),'k-','LineWidth',1.4); hold on;
        plot(tVn,nr*scale(j),'.','Color',cRaw,'MarkerSize',3);
        plot(tF,fe*scale(j),'-','Color',cFilter,'LineWidth',1);
        if j<=2
            plot(tDr,drXY(:,j),'--','Color',cDr,'LineWidth',1.4);
            legend('Ground truth','VN','Filtered','Encoder + VN heading','Location','best');
        else
            legend('Ground truth','VN','Filtered','Location','best');
        end
        grid on; xlabel('Time (s)'); ylabel(labels{j}); title(labels{j});
    end
    axesState(5)=subplot(3,2,5);
    plot(truth.E(:,1),truth.E(:,2),'k-','LineWidth',1.4); hold on;
    plot(tE,vE,'.','Color',cRaw,'MarkerSize',3);
    grid on; xlabel('Time (s)'); ylabel('Speed (m/s)');
    title('Encoder speed (raw measurement)');
    legend('Ground truth','Encoder','Location','best'); linkaxes(axesState,'x');
    subplot(3,2,6); axis off;
    text(0,0.9,{'Published: x, y, heading, angular velocity.'; ...
        'Velocity is estimated internally, but absent from F.'; ...
        'Purple: integral of raw E speed + raw VN heading.'; ...
        'Truth supplies only its initial position.'; ...
        'Heading interpolation is offline; no extrapolation.'; ...
        'Last F estimate retained at each timestamp.'; ...
        'RMSE tables appear in the Command Window.'}, ...
        'Units','normalized','VerticalAlignment','top','FontSize',10);

    figures(3)=figure('Name','Errors relative to ground truth','Color','w');
    axesError=gobjects(6,1);
    for j=1:6
        axesError(j)=subplot(3,2,j);
        if j<=4
            plot(tVn,rawError(:,j)*scale(j),'.','Color',cRaw,'MarkerSize',3); hold on;
            plot(tF,filterError(:,j)*scale(j),'-','Color',cFilter,'LineWidth',1);
            ylabel([labels{j},' error']);
            if j<=2
                plot(tDr,drError(:,j),'--','Color',cDr,'LineWidth',1.2);
                title(sprintf('RMSE: VN %.3g | integrated %.3g | filter %.3g', ...
                    positionMetrics.VN_RMSE_m(j),positionMetrics.EncoderHeading_RMSE_m(j), ...
                    positionMetrics.Filtered_RMSE_m(j)));
                legend('VN error','Filtered error','Encoder + VN heading error','Location','best');
            else
                title(sprintf('RMSE: VN %.4g | filter %.4g',rawRMSE(j),filterRMSE(j)));
                legend('VN error','Filtered error','Location','best');
            end
        elseif j==5
            plot(tE,encoderError,'.','Color',cRaw,'MarkerSize',3); hold on;
            ylabel('Speed error (m/s)'); title(sprintf('Encoder RMSE: %.4g m/s',encoderRMSE));
        else
            plot(tVn,hypot(rawError(:,1),rawError(:,2)),'.','Color',cRaw,'MarkerSize',3); hold on;
            plot(tF,hypot(filterError(:,1),filterError(:,2)),'-','Color',cFilter,'LineWidth',1);
            plot(tDr,hypot(drError(:,1),drError(:,2)),'--','Color',cDr,'LineWidth',1.2);
            ylabel('2D position error (m)');
            title(sprintf('RMSE: VN %.3g | integrated %.3g | filter %.3g', ...
                positionMetrics.VN_RMSE_m(3),positionMetrics.EncoderHeading_RMSE_m(3), ...
                positionMetrics.Filtered_RMSE_m(3)));
            legend('VN error','Filtered error','Encoder + VN heading error','Location','best');
        end
        limits=xlim; plot(limits,[0,0],'k:','HandleVisibility','off');
        grid on; xlabel('Time (s)');
    end
    linkaxes(axesError,'x');
end
results=struct('metrics',metrics,'positionMetrics',positionMetrics, ...
    'encoderMetrics',encoderMetrics,'truth',truth,'noisy',noisy,'filtered',filt, ...
    'rawError',rawError,'filterError',filterError,'encoderError',encoderError, ...
    'figures',figures);
results.deadReckoning=struct('time',tDr,'xy',drXY,'error',drError, ...
    'initialXY',initialXY,'heading',headingDr,'velocity',vDr);
end

function r=positionRMSE(e)
r=[sqrt(mean(e(:,1).^2));sqrt(mean(e(:,2).^2));sqrt(mean(sum(e.^2,2)))];
end

function data=readPackets(path,filtered)
[fid,msg]=fopen(path,'r');
if fid<0, error('Cannot open %s: %s',path,msg); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
lines=textscan(fid,'%s','Delimiter','\n','Whitespace',''); lines=lines{1};
n=numel(lines); VN=zeros(n,5); E=zeros(n,2); F=zeros(n,5);
statuses=cell(n,1); nv=0; ne=0; nf=0; skipped=0;
numberPattern='^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$';
for k=1:n
    s=strtrim(lines{k});
    if isempty(s) || s(1)=='#' || s(1)=='%', continue; end
    tokens=regexp(s,'\s+','split'); tag=upper(tokens{1});
    if filtered && ~strcmp(tag,'F')
        if any(strcmp(tag,{'VN','E','G','O'}))
            error('Expected F estimates in %s, line %d; found %s.',path,k,tag);
        end
        skipped=skipped+1; continue;
    end
    if ~filtered && ~any(strcmp(tag,{'VN','E'}))
        error('Unexpected packet type in %s, line %d: %s',path,k,tag);
    end
    if strcmp(tag,'E'), count=2; else, count=5; end
    ok=numel(tokens)>=count+1;
    if ~strcmp(tag,'F'), ok=ok && numel(tokens)==count+1; end
    v=zeros(1,count);
    if ok
        for j=1:count
            token=tokens{j+1};
            if isempty(regexp(token,numberPattern,'once')), ok=false; break; end
            v(j)=str2double(token);
            if ~isfinite(v(j)), ok=false; break; end
        end
    end
    status='';
    if ok && strcmp(tag,'F') && numel(tokens)>count+1
        tail=tokens(count+2:end);
        for j=1:numel(tail)
            if isempty(regexp(tail{j},'^[A-Za-z_][A-Za-z0-9_:-]*$','once'))
                ok=false; break;
            end
        end
        status=strjoin(tail,' ');
    end
    if ~ok, error('Malformed/nonfinite packet in %s, line %d.',path,k); end
    switch tag
        case 'VN', nv=nv+1; VN(nv,:)=v;
        case 'E', ne=ne+1; E(ne,:)=v;
        case 'F', nf=nf+1; F(nf,:)=v; statuses{nf}=status;
    end
end
[VN,dv]=canonical(VN(1:nv,:));
[E,de]=canonical(E(1:ne,:));
[F,df,idx]=canonical(F(1:nf,:));
statuses=statuses(1:nf); statuses=statuses(idx);
data=struct('VN',VN,'E',E,'F',F,'status',{statuses}, ...
    'skipped',skipped,'duplicates',dv+de+df);
end

function [a,removed,idx]=canonical(a)
oldN=size(a,1); idx=zeros(0,1);
if ~isempty(a)
    [~,idx]=unique(a(:,1),'last');
    a=a(idx,:);
end
removed=oldN-size(a,1);
end

function requireStream(a,name)
if size(a,1)<2, error('Need at least two distinct timestamps for %s.',name); end
end

function yi=interpolate(t,y,tq,isAngle)
if isAngle, y=unwrap(y); end
yi=interp1(t,y,tq,'linear',NaN);
end

function d=difference(a,b,isAngle)
d=a-b;
if isAngle, d=atan2(sin(d),cos(d)); end
end
