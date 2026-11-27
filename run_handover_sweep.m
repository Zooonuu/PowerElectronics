function summary = run_handover_sweep(durations_ms)
% R2024b companion to the original charger_simulink package.
% Run from charger_simulink: run_handover_sweep
% Optional subset: run_handover_sweep([25 100])
% All experiments use 0.30-0.80 s; existing 50 ms data are copied, not overwritten.
if nargin<1, durations_ms=[25 50 100 200]; end
validateattributes(durations_ms,{'numeric'},{'vector','finite','positive','integer','nonempty'});
assert(all(ismember(durations_ms,[25 50 100 200])),'Use 25, 50, 100, or 200 ms.');
durations_ms=unique(durations_ms(:)');
root=fileparts(mfilename('fullpath'));
if ~isfile(fullfile(root,'build_charger_models.m'))
    root=pwd;
end
assert(isfile(fullfile(root,'build_charger_models.m')) && isfile(fullfile(root,'run_charger_models.m')) && isfile(fullfile(root,'compare_charger_cases.m')),'Place this file in charger_simulink, next to the original three .m scripts.');
original=fullfile(root,'PSFB_CLLLC_models','data');
if any(durations_ms==50)
    assert(isfile(fullfile(original,'Case_A.csv')) && isfile(fullfile(original,'Case_B.csv')),'Original 50 ms Case_A.csv and Case_B.csv not found. Restore them or run a subset without 50.');
end
for names={'Case_A','Case_B'}
    name=names{1};
    if bdIsLoaded(name)
        assert(strcmp(get_param(name,'Dirty'),'off'),'Save or close modified model %s first.',name);
        close_system(name,0);
    end
end
old=pwd; cleanup=onCleanup(@()cd(old)); %#ok<NASGU>
addpath(root);
base=fullfile(root,['Handover_sweep_' datestr(now,'yyyymmdd_HHMMSS')]);
assert(~isfolder(base),'Sweep folder already exists; try again.'); mkdir(base);
summary=table; extra=table;
t0=0.30; t1=0.80;
for ms=durations_ms
    work=fullfile(base,sprintf('handover_%03dms',ms)); mkdir(work); cd(work);
    data=fullfile(work,'PSFB_CLLLC_models','data'); mkdir(data);
    fprintf('\n=== Handover %d ms ===\n',ms);
    if ms==50
        for names={'Case_A','Case_B'}
            f=[names{1} '.csv']; copyfile(fullfile(original,f),fullfile(data,f));
        end
    else
        build_charger_models({'Case_A','Case_B'},struct('nC',1.8,'tEnable',t0,'tHandover',ms/1000,'stop',t1));
        for names={'Case_A','Case_B'}
            name=names{1}; run_charger_models(name);
            if bdIsLoaded(name), close_system(name,0); end
        end
    end
    % Validate command timing and initial condition before interpreting performance.
    A=readtable(fullfile(data,'Case_A.csv')); B=readtable(fullfile(data,'Case_B.csv'));
    [ea,preA]=extraMetrics(A,ms,t0,t1,'A');
    [eb,preB]=extraMetrics(B,ms,t0,t1,'B');
    assert(all(abs(preA-preB)<[0.1 0.1 0.2]),'Case A/B pre-handover averages differ; inspect the files.');
    clear A B
    m=compare_charger_cases(data,t0,t1);
    m=addvars(m,repmat(ms,height(m),1),'Before',1,'NewVariableNames','Handover_ms');
    summary=[summary;m]; %#ok<AGROW>
    ea.Handover_ms=ms; eb.Handover_ms=ms;
    extra=[extra;ea;eb]; %#ok<AGROW>
    writetable(summary,fullfile(base,'Handover_sweep_metrics.csv'));
    writetable(extra,fullfile(base,'Handover_sweep_extra.csv'));
    fprintf('Saved progress in %s\n',base);
end
cd(base);
f=figure('Color','w','Name','Handover duration sweep'); tiledlayout(2,2,'TileSpacing','compact');
keys={'Max_total_error_A','Max_voltage_error_V','RMSE_total_A','Settling_total_ms'};
ylabs={'Max mean-current error (A)','Max voltage deviation (V)','Current RMSE (A)','Total-current settling (ms)'};
for j=1:4
    q=summary(strcmp(summary.Metric,keys{j}),:); nexttile;
    plot(q.Handover_ms,q.Case_A,'-o','LineWidth',1.5); hold on;
    plot(q.Handover_ms,q.Case_B,'-s','LineWidth',1.5); grid on;
    xlabel('Command ramp duration (ms)'); ylabel(ylabs{j}); xticks(durations_ms);
    legend('Case A','Case B','Location','best');
end
exportgraphics(f,fullfile(base,'Handover_sweep_comparison.png'),'Resolution',220);
f2=figure('Color','w','Name','Current sharing completion');
for caseName={'A','B'}
    q=extra(strcmp(extra.Case,caseName{1}),:);
    plot(q.Handover_ms,q.Sharing_settling_ms,'-o','LineWidth',1.5); hold on;
end
xlabel('Command ramp duration (ms)'); ylabel('Sharing settling time (ms)'); xticks(durations_ms); grid on;
legend('Case A','Case B','Location','best');
title('Both mean module currents within 25 +/- 0.5 A');
exportgraphics(f2,fullfile(base,'Handover_sharing_settling.png'),'Resolution',220);
disp(summary); disp(extra); fprintf('\nDONE: %s\n',base);
end

function [q,pre]=extraMetrics(T,ms,t0,t1,caseName)
t=T.time(:); dt=median(diff(t));
assert(all(diff(t)>0) && abs(dt-1e-6)<1e-8,'Expected original 1 us logging interval.');
assert(abs(t(end)-t1)<dt,'Expected 0.8 s simulation.');
assert(all(abs(T.V_in-400)<0.1),'Expected 400 V input.');
check=t>=t0+0.001 & t<=t0+ms/1000-0.001;
assert(max(abs(T.I_CLLLC_ref(check)-25*(t(check)-t0)/(ms/1000)))<0.15,'CLLLC reference ramp differs from requested experiment.');
checkEnd=t>=t0+ms/1000+0.001;
assert(all(abs(T.I_CLLLC_ref(checkEnd)-25)<0.01),'Unexpected CLLLC final reference.');
if strcmp(caseName,'A')
    checkA=t>=t0;
    assert(max(abs(T.I_PSFB_ref(checkA)+T.I_CLLLC_ref(checkA)-50))<0.15,'Unexpected Case A reference complement.');
end
n=max(1,round(100e-6/dt));
ip=movmean(T.I_PSFB,[n-1 0]); ic=movmean(T.I_CLLLC,[n-1 0]);
v=movmean(T.V_out,[n-1 0]);
idx=t>=t0 & t<=t1; preidx=t>=t0-.02 & t<t0;
pre=[mean(ip(preidx)),mean(ic(preidx)),mean(v(preidx))];
assert(all(abs(pre-[50 0 200])<[1 .5 2]),'Pre-handover condition is not 50 A / 0 A / 200 V.');
te=t(idx); good=abs(ip(idx)-25)<=0.5 & abs(ic(idx)-25)<=0.5;
last=find(~good,1,'last');
if isempty(last), sharing=0; elseif last==numel(te), sharing=NaN; else, sharing=(te(last+1)-t0)*1000; end
% NaN means not settled by the end of the observation window.
ss=t>=t1-.01 & t<=t1;
q=table({caseName},ms,pre(1),pre(2),pre(3),min(v(idx)),max(v(idx)), ...
    sharing,mean(ip(ss)),mean(ic(ss)),max(T.D_PSFB(idx)),max(T.D_CLLLC(idx)), ...
    'VariableNames',{'Case','Handover_ms','Pre_PSFB_A','Pre_CLLLC_A','Pre_Vout_V','Min_Vout_V','Max_Vout_V', ...
    'Sharing_settling_ms','Final_PSFB_A','Final_CLLLC_A','Max_D_PSFB','Max_D_CLLLC'});
end
