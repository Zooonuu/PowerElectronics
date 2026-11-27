function out = run_charger_models(whichModels, stopTime)
% Run generated switching models, export measured timeseries to CSV.
% run_charger_models('PSFB_single_4ohm',0.005) % smoke test (not steady state)
% run_charger_models('all')                 % full run
% run_charger_models({'Case_A','Case_B'})   % run and compare
if nargin<1 || isempty(whichModels), whichModels={'Case_A','Case_B'}; end
if nargin<2, stopTime=[]; end
if ischar(whichModels)||isstring(whichModels), whichModels=cellstr(whichModels); end
if numel(whichModels)==1 && strcmpi(whichModels{1},'all')
    whichModels={'PSFB_single_4ohm','PSFB_single_8ohm','CLLLC_single_8ohm','Parallel_Base','Case_A','Case_B'};
end
folder=fullfile(pwd,'PSFB_CLLLC_models'); dataDir=fullfile(folder,'data');
if ~exist(dataDir,'dir'), mkdir(dataDir); end
out=struct;
for k=1:numel(whichModels)
    name=char(whichModels{k}); path=fullfile(folder,[name '.slx']);
    assert(exist(path,'file')==2,'Generate %s first.',name);
    if ~bdIsLoaded(name), load_system(path); end
    if ~strcmpi(get_param(name,'FileName'),path), error('Different model with the same name is open: %s',name); end
    in=Simulink.SimulationInput(name);
    if ~isempty(stopTime), in=in.setModelParameter('StopTime',num2str(stopTime,17)); end
    fprintf('Simulating %s ...\n',name);
    result=sim(in);
    out.(name)=result;
    tab=tableFromOutput(result);
    suffix=''; if ~isempty(stopTime), suffix=sprintf('_test_%g_s',stopTime); end
    writetable(tab,fullfile(dataDir,[name suffix '.csv']));
    fprintf('Exported %s (%d samples)\n',name,height(tab));
    plotOne(tab,name);
    if isempty(stopTime)
        plotPath=fullfile(dataDir,[name '.png']);
        exportgraphics(gcf,plotPath,'Resolution',180);
    end
end
if isempty(stopTime) && any(strcmp(whichModels,'Case_A')) && any(strcmp(whichModels,'Case_B'))
    compare_charger_cases(dataDir);
end
end

function T=tableFromOutput(o)
ref=o.get('V_out'); t=double(ref.Time(:));
T=table(t,'VariableNames',{'time'});
vars={'V_out','I_PSFB','I_CLLLC','I_load','I_PSFB_ref','I_CLLLC_ref','V_in','I_in_PSFB','I_in_CLLLC','CLLLC_enable','I_CLLLC_meas','I_Lrp','D_PSFB','D_CLLLC'};
found=who(o);
for k=1:numel(vars)
    v=vars{k}; if ~any(strcmp(found,v)), continue; end
    z=o.get(v); tz=double(z.Time(:)); d=double(squeeze(z.Data)); d=d(:);
    if isequal(tz,t), T.(v)=d;
    else
        [tz,j]=unique(tz,'last'); d=d(j);
        if numel(tz)==1, T.(v)=repmat(d(1),size(t));
        else, T.(v)=interp1(tz,d,t,'previous','extrap'); end
    end
end
T.I_total=T.I_PSFB+T.I_CLLLC;
end

function plotOne(T,name)
figure('Name',name,'Color','w'); tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(T.time,T.V_out); yline(200,'--'); ylabel('V_{out} (V)'); grid on; title(strrep(name,'_',' '));
nexttile; plot(T.time,[T.I_PSFB T.I_CLLLC T.I_load]); ylabel('Current (A)'); legend('PSFB','CLLLC','Load','Location','best'); grid on;
nexttile; plot(T.time,[T.I_PSFB_ref T.I_CLLLC_ref]); ylabel('Reference (A)'); xlabel('Time (s)'); legend('PSFB ref','CLLLC ref','Location','best'); grid on;
end
