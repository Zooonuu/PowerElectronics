function metrics = compare_charger_cases(dataDir,t0,t1)
% Compare measured Case A/B data. No simulated numbers are fabricated.
% Default comparison window is 0.05 to 0.15 s; adjust only identically.
if nargin<1 || isempty(dataDir), dataDir=fullfile(pwd,'PSFB_CLLLC_models','data'); end
if nargin<2, t0=0.05; end
if nargin<3, t1=0.15; end
A=readtable(fullfile(dataDir,'Case_A.csv')); B=readtable(fullfile(dataDir,'Case_B.csv'));
assert(t0<t1 && t0>min(A.time) && t0>min(B.time) && t1<=min(max(A.time),max(B.time)), 'Invalid common comparison window.');
assert(all(ismember({'time','I_PSFB','I_CLLLC','I_load','V_out'},A.Properties.VariableNames)) && ...
       all(ismember({'time','I_PSFB','I_CLLLC','I_load','V_out'},B.Properties.VariableNames)), 'Missing measured channels.');
[xa,ap]=measure(A,t0,t1); [xb,bp]=measure(B,t0,t1);
labels={'Max_total_error_A';'RMSE_total_A';'IAE_total_As';'Max_voltage_error_V'; ...
    'Settling_total_ms';'Steady_PSFB_error_A';'Steady_CLLLC_error_A';'Max_raw_total_error_A';'Max_raw_voltage_error_V'};
imp=nan(size(xa)); valid=isfinite(xa)&isfinite(xb)&abs(xa)>eps; imp(valid)=100*(xa(valid)-xb(valid))./xa(valid);
metrics=table(labels,xa,xb,imp,'VariableNames',{'Metric','Case_A','Case_B','Improvement_pct'});
disp(metrics); writetable(metrics,fullfile(dataDir,'Case_AB_metrics.csv'));
figure('Name','Case A vs B','Color','w'); tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(ap.t,ap.total,bp.t,bp.total); yline(50,'--'); xline(t0,':'); xline(t1,':'); ylabel('I_{total} (A)'); legend('Case A','Case B','Target','Location','best'); grid on; title('Causal 100 us moving average');
nexttile; plot(ap.t,ap.v,bp.t,bp.v); yline(200,'--'); xline(t0,':'); xline(t1,':'); ylabel('V_{out} (V)'); grid on;
nexttile; plot(ap.t,50-ap.total,bp.t,50-bp.total); xline(t0,':'); xline(t1,':'); ylabel('50 - I_{total} (A)'); xlabel('Time (s)'); grid on;
exportgraphics(gcf,fullfile(dataDir,'Case_AB_comparison.png'),'Resolution',200);
end

function [x,s]=measure(T,t0,t1)
t=T.time(:); assert(all(diff(t)>0),'Time must be strictly increasing.');
Ts=median(diff(t)); n=max(1,round(100e-6/Ts));
assert(Ts<=100e-6,'Sample interval too coarse for 100 us moving average.');
raw=T.I_PSFB+T.I_CLLLC;
% Identical causal window for both cases; no centered look-ahead.
s.t=t; s.total=movmean(raw,[n-1 0]); s.v=movmean(T.V_out,[n-1 0]);
ip=movmean(T.I_PSFB,[n-1 0]); ic=movmean(T.I_CLLLC,[n-1 0]);
idx=t>=t0 & t<=t1; pre=t>=max(t(1),t0-.005) & t<t0;
if ~any(pre) || abs(mean(ip(pre))-50)>2.5 || abs(mean(ic(pre)))>1.25 || abs(mean(s.v(pre))-200)>10
    warning('Pre-handover initial condition is not yet 50A / 0A / 200V. Do not claim a fair handover comparison.');
end
if any(abs(ip(pre)-50)>5), warning('PSFB current is not settled before handover.'); end
e=50-s.total(idx); ev=200-s.v(idx); te=t(idx); dur=te(end)-te(1);
assert(numel(te)>1 && dur>0,'Need at least two samples in window.');
lastBad=find(abs(s.total(t>=t0)-50)>1,1,'last'); after=t(t>=t0);
if isempty(lastBad), settle=0;
elseif lastBad>=numel(after), settle=NaN;
else, settle=(after(lastBad+1)-t0)*1e3;
end
ss=t>=max(t0,t(end)-.01);
x=[max(abs(e));sqrt(trapz(te,e.^2)/dur);trapz(te,abs(e));max(abs(ev));settle; ...
   abs(mean(ip(ss))-25);abs(mean(ic(ss))-25);max(abs(50-raw(idx)));max(abs(200-T.V_out(idx)))];
end
