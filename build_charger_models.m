function models = build_charger_models(whichModels, overrides)
% BUILD_CHARGER_MODELS  Rebuild PSFB/CLLLC switching models in R2024b SPS.
%   build_charger_models              % all six models
%   build_charger_models('Case_A')    % one model
%   build_charger_models('all',struct('nC',1.8)) % optional tuning
% Models are saved to ./PSFB_CLLLC_models. No PSIM license needed.
% Reconstruction, not a binary PSIM converter. See README_KO.txt.
if nargin < 1 || isempty(whichModels), whichModels = 'all'; end
if nargin < 2, overrides = struct; end
p = defaults();
for f = fieldnames(overrides)'
    k = f{1}; assert(isfield(p,k),'Unknown parameter: %s',k); p.(k)=overrides.(k);
end
assert(p.dt>0 && p.fP>0 && p.fC>0);
assert(abs(1/p.fP/p.dt-round(1/p.fP/p.dt))<1e-7 && ...
       abs(1/p.fC/p.dt-round(1/p.fC/p.dt))<1e-7, ...
       'dt must divide both switching periods.');
load_system('simulink'); load_system('sps_lib');
try, load_system('powerlib'); catch, end
lib=struct;
lib.dc=locate('DC Voltage Source',{'sps_lib/Sources/DC Voltage Source','powerlib/Electrical Sources/DC Voltage Source'});
lib.bridge=locate('Universal Bridge',{'sps_lib/Power Electronics/Universal Bridge','powerlib/Power Electronics/Universal Bridge'});
lib.rlc=locate('Series RLC Branch',{'sps_lib/Passives/Series RLC Branch','powerlib/Elements/Series RLC Branch'});
lib.xf=locate('Linear Transformer',{'sps_lib/Power Grid Elements/Linear Transformer','powerlib/Elements/Linear Transformer'});
lib.v=locate('Voltage Measurement',{'sps_lib/Sensors and Measurements/Voltage Measurement','powerlib/Measurements/Voltage Measurement'});
lib.i=locate('Current Measurement',{'sps_lib/Sensors and Measurements/Current Measurement','powerlib/Measurements/Current Measurement'});
lib.gnd=locate('Ground',{'sps_lib/Utilities/Ground','sps_lib/Passives/Ground','powerlib/Elements/Ground'});
lib.pg=locate('powergui',{'sps_lib/powergui','powerlib/powergui'});
names={'PSFB_single_4ohm','PSFB_single_8ohm','CLLLC_single_8ohm','Parallel_Base','Case_A','Case_B'};
if ischar(whichModels)||isstring(whichModels), whichModels=cellstr(whichModels); end
if numel(whichModels)==1 && strcmpi(whichModels{1},'all'), whichModels=names; end
outDir=fullfile(pwd,'PSFB_CLLLC_models'); if ~exist(outDir,'dir'), mkdir(outDir); end
models=cell(0,1);
for k=1:numel(whichModels)
    name=char(whichModels{k}); mode=find(strcmpi(name,names),1); assert(~isempty(mode),'Unknown model: %s',name); name=names{mode};
    if bdIsLoaded(name), error('Close the open model %s before rebuilding (save your changes first).',name); end
    dest=fullfile(outDir,[name '.slx']);
    if exist(dest,'file'), copyfile(dest,[dest '.bak']); end
    fprintf('\nBuilding %s ...\n',name);
    new_system(name); open_system(name);
    try
        buildOne(name,mode,p,lib);
        save_system(name,dest);
        fprintf('Saved: %s\n',dest);
        models{end+1,1}=dest; %#ok<AGROW>
    catch ME
        fprintf(2,'Build failed at %s. The partial model remains open for diagnosis.\n',name);
        rethrow(ME);
    end
    if k<numel(whichModels), close_system(name,0); end
end
save(fullfile(outDir,'design_parameters.mat'),'p');
fprintf('\nGenerated %d model(s). Last model remains open.\n',numel(models));
fprintf('Validate single-module models before relying on Case A/B.\n');
end

function p=defaults()
% Confirmed source values and explicit reconstruction assumptions.
p.Vin=400; p.Vout=200; p.Rparallel=4; p.Rsingle=8;
p.fP=10000; p.fC=100000; p.dt=50e-9; p.stop=0.3;
p.nP=1; p.nC=2; p.LoutP=2e-3; p.Csingle=1e-3; p.Cbus=2e-3;
p.Crp=309e-9; p.Lrp=8.2e-6; p.Lm=41e-6; p.Lrs=2.05e-6; p.Crs=1.236e-6;
p.Rp=1e-6; p.Rs=1e-6; p.LlkP=1e-6; p.LlkS=1e-6;
% Assumptions / initial tuning, not values extracted from final PSIM binaries.
p.Ron=1e-3; p.Vf=0; p.Rsn=1e5; p.ESR=1e-3; p.RoutP=0.01; p.Rres=0.005;
p.deadP=1e-6; p.deadC=0.2e-6; p.KpP=0.003; p.KiP=0.6;
p.KpC=0.002; p.KiC=0.3; p.tauP=0.3e-3; p.tauC=0.2e-3;
p.dMax=0.95; p.ImaxP=50; p.ImaxC=25;
p.tStart=0.01; p.tRamp=0.02; p.tEnable=0.05; p.tHandover=0.05;
p.logDecimation=20; p.RisolationRef=1e9;
end

function buildOne(m,mode,p,L)
useP=mode~=3; useC=mode>=3; parallel=mode>=4;
R=p.Rsingle; if mode==1||parallel, R=p.Rparallel; end
C=p.Csingle; if parallel, C=p.Cbus; end
set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep',num(p.dt),'StopTime',num(p.stop),'ReturnWorkspaceOutputs','on', ...
    'SaveTime','off','SaveOutput','off','SignalLogging','off');
pg=put(L.pg,m,'powergui',[25 25 125 65]);
maskset(pg,{'SimulationMode','SimulationType'},'simulation (type|mode)','Discrete');
maskset(pg,{'SampleTime'},'sample time',num(p.dt));
src=put(L.dc,m,'DC_400V',[40 235 95 325]);
maskset(src,{'Amplitude','Voltage'},'amplitude|voltage',num(p.Vin));
sp=pair(src); g=put(L.gnd,m,'Input_reference',[45 370 75 400]); gp=allphys(g); wire(m,sp(2),gp(1));
% Explicit high-resistance numerical reference: avoid tying isolated returns.
ref=rlc(m,L,'Isolation_reference','R',p.RisolationRef,0,0,[180 710 275 740]);
refp=pair(ref); wire(m,sp(2),refp(1));
if useP, P=moduleP(m,L,p,165); wire(m,sp(1),P.dc(1)); wire(m,sp(2),P.dc(2)); end
if useC, Q=moduleC(m,L,p,470); wire(m,sp(1),Q.dc(1)); wire(m,sp(2),Q.dc(2)); end
% Common output network. Capacitors of both modules are represented by one bus C.
cap=rlc(m,L,'Bus_capacitor','RC',p.ESR,0,C,[1200 355 1270 395]);
load=rlc(m,L,'Load_R','R',R,0,0,[1480 355 1550 395]);
loadI=put(L.i,m,'I_load',[1360 300 1410 330]);
v=put(L.v,m,'V_out',[1315 420 1365 470]);
cp=pair(cap); rp=pair(load); lp=pair(loadI); vp=pair(v);
wire(m,cp(1),lp(1)); wire(m,lp(2),rp(1)); wire(m,cp(2),rp(2));
wire(m,cp(1),vp(1)); wire(m,cp(2),vp(2)); wire(m,cp(2),refp(2));
if useP, wire(m,cp(1),P.out(1)); wire(m,cp(2),P.out(2)); end
if useC, wire(m,cp(1),Q.out(1)); wire(m,cp(2),Q.out(2)); end
% Measurement of actual DC input (not a fixed label).
vin=put(L.v,m,'V_in',[110 430 155 480]); z=pair(vin); wire(m,sp(1),z(1)); wire(m,sp(2),z(2));
% Controller with explicit switching gate signals and sampled PI integrators.
clk=put('simulink/Sources/Digital Clock',m,'Clock',[30 830 90 860]); set_param(clk,'SampleTime',num(p.dt));
ctl=put('simulink/User-Defined Functions/MATLAB Function',m,'Current_handover_and_PWM',[430 820 700 1000]);
r=sfroot; ch=r.find('-isa','Stateflow.EMChart','Path',ctl); assert(~isempty(ch),'Cannot find MATLAB Function chart.'); ch.Script=controller(p,mode);
for k=1:2
    if k==1, active=useP; if active, meas=P.i; end; tau=p.tauP; tag='PSFB';
    else, active=useC; if active, meas=Q.i; end; tau=p.tauC; tag='CLLLC'; end
    if active
        fl=put('simulink/Discrete/Discrete Transfer Fcn',m,['Filter_' tag],[215 850+80*k 340 880+80*k]);
        a=p.dt/(tau+p.dt); set_param(fl,'Numerator',num(a),'Denominator',sprintf('[1 %.17g]',-(1-a)),'SampleTime',num(p.dt));
        sig(m,meas,1,fl,1);
    else
        fl=put('simulink/Sources/Constant',m,['Zero_' tag],[235 850+80*k 285 880+80*k]); set_param(fl,'Value','0','SampleTime',num(p.dt));
    end
    sig(m,fl,1,ctl,k+1);
    if k==2, logSignal(m,fl,1,'I_CLLLC_meas',p,1550,970); end
end
sig(m,clk,1,ctl,1);
if useP, sig(m,ctl,1,P.bridge,1); logSignal(m,P.i,1,'I_PSFB',p,1550,75); logSignal(m,P.inI,1,'I_in_PSFB',p,1550,120); else, terminate(m,ctl,1,'Unused_gate_P',[760 820 780 840]); zeroLog(m,'I_PSFB',p,1550,75); end
if useC, sig(m,ctl,2,Q.bridge,1); logSignal(m,Q.i,1,'I_CLLLC',p,1550,170); logSignal(m,Q.inI,1,'I_in_CLLLC',p,1550,215); logSignal(m,Q.resI,1,'I_Lrp',p,1550,260); else, terminate(m,ctl,2,'Unused_gate_C',[760 855 780 875]); zeroLog(m,'I_CLLLC',p,1550,170); end
logSignal(m,loadI,1,'I_load',p,1630,310); logSignal(m,v,1,'V_out',p,1630,440); logSignal(m,vin,1,'V_in',p,1630,490);
labels={'I_PSFB_ref','I_CLLLC_ref','CLLLC_enable','D_PSFB','D_CLLLC'};
for k=1:numel(labels), logSignal(m,ctl,k+2,labels{k},p,1000,820+45*k); end
% Scope with three inputs so that module currents and load current remain distinct.
sc=put('simulink/Sinks/Scope',m,'Current_scope',[1700 85 1780 190]); set_param(sc,'NumInputPorts','3');
if useP, sig(m,P.i,1,sc,1); else, sig(m,[m '/Zero_I_PSFB'],1,sc,1); end
if useC, sig(m,Q.i,1,sc,2); else, sig(m,[m '/Zero_I_CLLLC'],1,sc,2); end
sig(m,loadI,1,sc,3);
sv=put('simulink/Sinks/Scope',m,'Voltage_scope',[1700 550 1770 600]); sig(m,v,1,sv,1);
try, set_param(m,'ZoomFactor','FitSystem'); catch, end
end

function P=moduleP(m,L,p,y)
P.bridge=bridge(m,L,'PSFB_full_bridge',false,p,[245 y 325 y+100]); [a,d]=bridgePorts(P.bridge);
P.inI=put(L.i,m,'PSFB_input_current',[105 y 155 y+30]); ip=pair(P.inI); wire(m,ip(2),d(1)); P.dc=[ip(1) d(2)];
T=xf(m,L,'PSFB_transformer',p.nP,p.fP,p,[465 y 535 y+100]); [t1,t2]=xfPorts(T);
wire(m,a(1),t1(1)); wire(m,a(2),t1(2));
rect=bridge(m,L,'PSFB_diode_rectifier',true,p,[665 y 745 y+100]); [ra,rd]=bridgePorts(rect); wire(m,t2(1),ra(1)); wire(m,t2(2),ra(2));
lo=rlc(m,L,'PSFB_Lout','RL',p.RoutP,p.LoutP,0,[825 y+10 905 y+45]); lp=pair(lo); wire(m,rd(1),lp(1));
P.i=put(L.i,m,'PSFB_bus_current',[1010 y+10 1070 y+45]); oi=pair(P.i); wire(m,lp(2),oi(1)); P.out=[oi(2) rd(2)];
end

function Q=moduleC(m,L,p,y)
Q.bridge=bridge(m,L,'CLLLC_full_bridge',false,p,[245 y 325 y+100]); [a,d]=bridgePorts(Q.bridge);
Q.inI=put(L.i,m,'CLLLC_input_current',[105 y 155 y+30]); ip=pair(Q.inI); wire(m,ip(2),d(1)); Q.dc=[ip(1) d(2)];
cr=rlc(m,L,'CLLLC_Crp','C',0,0,p.Crp,[355 y 415 y+30]); cp=pair(cr); wire(m,a(1),cp(1));
lr=rlc(m,L,'CLLLC_Lrp','RL',p.Rres,p.Lrp,0,[440 y 510 y+30]); lp=pair(lr); wire(m,cp(2),lp(1));
Q.resI=put(L.i,m,'CLLLC_resonant_current',[535 y 585 y+30]); ri=pair(Q.resI); wire(m,lp(2),ri(1));
lm=rlc(m,L,'CLLLC_Lm','L',0,p.Lm,0,[555 y+120 620 y+150]); mp=pair(lm);
T=xf(m,L,'CLLLC_transformer',p.nC,p.fC,p,[640 y 710 y+100]); [t1,t2]=xfPorts(T);
wire(m,ri(2),t1(1)); wire(m,ri(2),mp(1)); wire(m,a(2),t1(2)); wire(m,a(2),mp(2));
ls=rlc(m,L,'CLLLC_Lrs','RL',p.Rres,p.Lrs,0,[740 y 805 y+30]); lsp=pair(ls); wire(m,t2(1),lsp(1));
cs=rlc(m,L,'CLLLC_Crs','C',0,0,p.Crs,[830 y 890 y+30]); csp=pair(cs); wire(m,lsp(2),csp(1));
rect=bridge(m,L,'CLLLC_diode_rectifier',true,p,[910 y 990 y+100]); [ra,rd]=bridgePorts(rect); wire(m,csp(2),ra(1)); wire(m,t2(2),ra(2));
Q.i=put(L.i,m,'CLLLC_bus_current',[1050 y 1110 y+30]); oi=pair(Q.i); wire(m,rd(1),oi(1)); Q.out=[oi(2) rd(2)];
end

function b=bridge(m,L,name,diodes,p,pos)
b=put(L.bridge,m,name,pos);
maskset(b,{'Arms','NumberOfArms','NumberOfBridgeArms'},'number of bridge arms','2');
if diodes, device='Diodes'; else, device='IGBT / Diodes'; end
maskset(b,{'PowerElectronicDevice','Device'},'power electronic device',device);
maskset(b,{'Rs','SnubberResistance'},'snubber resistance',num(p.Rsn));
maskset(b,{'Cs','SnubberCapacitance'},'snubber capacitance','inf');
maskset(b,{'Ron'},'(^|\W)Ron|internal resistance',num(p.Ron));
if diodes
    optional(b,{'Lon'},'(^|\W)Lon|internal inductance','0');
    optional(b,{'Vf','ForwardVoltage'},'forward voltage',num(p.Vf));
else
    optional(b,{'ForwardVoltages','Vf','VfVfd'},'forward voltages',sprintf('[%g %g]',p.Vf,p.Vf));
    optional(b,{'TfTt','SwitchingTimes'},'Tf.*Tt','[0 0]');
end
optional(b,{'Measurements'},'^measurements','None');
end

function b=xf(m,L,name,n,f,p,pos)
b=put(L.xf,m,name,pos);
maskset(b,{'Units'},'^units','SI');
maskset(b,{'ThreeWindings','ThreeWinding'},'three windings','off');
maskset(b,{'NominalPowerFrequency','NominalPower','Pnfn'},'nominal power.*frequency',sprintf('[10000 %.17g]',f));
maskset(b,{'Winding1','Winding1Parameters'},'winding 1 parameters',sprintf('[400 %.17g %.17g]',p.Rp,p.LlkP));
maskset(b,{'Winding2','Winding2Parameters'},'winding 2 parameters',sprintf('[%.17g %.17g %.17g]',400/n,p.Rs,p.LlkS));
% Explicit Lm is external in CLLLC; finite high Rm required with leakage L.
maskset(b,{'MagnetizationResistance','Magnetization','RmLm','MagnetizationParameters'},'magnetization resistance.*(inductance|reactance)','[1e8 inf]');
optional(b,{'Measurements'},'^measurements','None');
end

function b=rlc(m,L,name,kind,R,ind,C,pos)
b=put(L.rlc,m,name,pos); maskset(b,{'BranchType'},'branch type',kind);
if contains(kind,'R'), maskset(b,{'Resistance'},'^resistance',num(R)); end
if contains(kind,'L'), maskset(b,{'Inductance'},'^inductance',num(ind)); end
if contains(kind,'C'), maskset(b,{'Capacitance'},'^capacitance',num(C)); end
optional(b,{'Setx0','SetiL0'},'set the initial inductor current','off');
optional(b,{'Setx0','SetvC0'},'set the initial capacitor voltage','off');
optional(b,{'Measurements'},'^measurements','None');
end

function s=controller(p,mode)
% Generated MATLAB Function: fixed carrier frequencies; intra-bridge phase shift.
% Discrete PI updates once per carrier period, switching edges at dt.
s=sprintf([ ...
'function [gP,gC,rP,rC,en,dP,dC] = fcn(t,iP,iC)\n' ...
'%%#codegen\n' ...
'persistent zP zC hP hC rPh rCh lastP lastC\n' ...
'if isempty(zP)\n zP=0; zC=0; hP=0; hC=0; rPh=0; rCh=0; lastP=-1; lastC=-1;\nend\n' ...
'dt=%.17g; fP=%.17g; fC=%.17g; mode=%d;\n' ...
'kP=%.17g; kiP=%.17g; kC=%.17g; kiC=%.17g;\n' ...
'deadP=%.17g; deadC=%.17g; dmax=%.17g; maxP=%.17g; maxC=%.17g;\n' ...
'start=%.17g; ramp=%.17g; hand=%.17g; duration=%.17g;\n' ...
'a=min(1,max(0,(t-start)/ramp)); b=min(1,max(0,(t-hand)/duration));\n' ...
'enP=(mode~=3); enC=(mode==3 || mode==4 || ((mode==5 || mode==6) && t>=hand));\n' ...
'if mode==1\n rp=50*a; rc=0;\nelseif mode==2\n rp=25*a; rc=0;\nelseif mode==3\n rp=0; rc=25*a;\nelseif mode==4\n rp=25*a; rc=25*a;\nelseif mode==5\n rp=50*a-25*b; rc=25*b;\nelse\n rc=25*b; rp=50*a; if enC, rp=50-iC; end\nend\n' ...
'rp=min(maxP,max(0,rp)); rc=min(maxC,max(0,rc));\n' ...
'cycleP=floor(t*fP+1e-7); cycleC=floor(t*fC+1e-7);\n' ...
'if cycleP~=lastP\n lastP=cycleP; rPh=rp;\n if enP, [hP,zP]=piStep(rp-iP,zP,kP,kiP,1/fP,dmax); else, hP=0; zP=0; end\nend\n' ...
'if cycleC~=lastC\n lastC=cycleC; rCh=rc;\n if enC, [hC,zC]=piStep(rc-iC,zC,kC,kiC,1/fC,dmax); else, hC=0; zC=0; end\nend\n' ...
'gP=gates(t,fP,deadP,hP,enP,dt); gC=gates(t,fC,deadC,hC,enC,dt);\n' ...
'rP=rPh; rC=rCh; en=double(enC); dP=hP; dC=hC;\nend\n' ...
'function [u,z]=piStep(e,z,kp,ki,T,lim)\n' ...
'v=kp*e+z+ki*T*e;\n' ...
'if (v<=lim && v>=0) || (v>lim && e<0) || (v<0 && e>0), z=z+ki*T*e; end\n' ...
'u=min(lim,max(0,kp*e+z));\nend\n' ...
'function g=gates(t,f,dead,d,en,dt)\n' ...
'%% Universal Bridge forced-commutation order: Aupper, Alower, Bupper, Blower.\n' ...
'g=zeros(4,1); if ~en, return; end\n' ...
'N=round(1/(f*dt)); k=mod(round(t/dt),N); shift=round(d*N/2);\n' ...
'ka=k; kb=mod(k-shift,N); h=N/2; gap=max(1,ceil(dead/(2*dt)));\n' ...
'g(1)=double(ka>=gap && ka<h-gap); g(2)=double(ka>=h+gap && ka<N-gap);\n' ...
'g(3)=double(kb>=gap && kb<h-gap); g(4)=double(kb>=h+gap && kb<N-gap);\nend\n'], ...
p.dt,p.fP,p.fC,mode,p.KpP,p.KiP,p.KpC,p.KiC,p.deadP,p.deadC,p.dMax,p.ImaxP,p.ImaxC,p.tStart,p.tRamp,p.tEnable,p.tHandover);
end

function b=put(source,m,name,pos)
b=[m '/' name]; add_block(source,b,'Position',pos);
end
function path=locate(name,candidates)
for i=1:numel(candidates)
    try, get_param(candidates{i},'Handle'); path=candidates{i}; return; catch, end
end
for root={'sps_lib','powerlib'}
    try
        q=find_system(root{1},'LookUnderMasks','none','FollowLinks','off','Type','Block','Name',name);
        if ~isempty(q), [~,j]=min(cellfun(@length,q)); path=q{j}; return; end
    catch, end
end
error('SPS library block not found: %s. Open sps_lib and check installation.',name);
end
function maskset(b,keys,pattern,val)
if ~trymask(b,keys,pattern,val), error('Unable to set %s on %s. Inspect get_param(block,''DialogParameters'').',pattern,b); end
end
function optional(b,keys,pattern,val)
if ~trymask(b,keys,pattern,val), fprintf('Optional/default mask setting not applied: %s (%s)\n',b,pattern); end
end
function ok=trymask(b,keys,pattern,val)
ok=false; d=get_param(b,'DialogParameters'); names=fieldnames(d);
for k=1:numel(keys)
    j=find(strcmpi(keys{k},names),1); if ~isempty(j)
        try, set_param(b,names{j},val); ok=true; return; catch, end
    end
end
try
    n=get_param(b,'MaskNames'); prompts=get_param(b,'MaskPrompts');
    jj=find(~cellfun('isempty',regexpi(prompts,pattern)));
    for j=jj(:)'
        try, set_param(b,n{j},val); ok=true; return; catch, end
    end
catch, end
end
function a=allphys(b)
h=get_param(b,'PortHandles'); a=[h.LConn(:);h.RConn(:)]'; a=a(a>0);
end
function q=pair(b)
q=allphys(b); assert(numel(q)==2,'Expected two electrical terminals: %s',b);
xy=cell2mat(arrayfun(@(h)get_param(h,'Position'),q(:),'UniformOutput',false));
if rangeLocal(xy(:,1))>=rangeLocal(xy(:,2)), [~,j]=sortrows(xy,[1 2]); else, [~,j]=sortrows(xy,[2 1]); end
q=q(j);
end
function d=rangeLocal(x), d=max(x)-min(x); end
function [a,d]=bridgePorts(b)
% R2024b default orientation: phase terminals on left, DC +/- on right.
[a,d]=xfPorts(b);
end
function [a,d]=xfPorts(b)
h=get_param(b,'PortHandles'); a=h.LConn(:)'; d=h.RConn(:)';
assert(numel(a)==2 && numel(d)==2,'Expected two terminals on each side: %s',b);
a=sortY(a); d=sortY(d);
end
function q=sortY(q)
xy=cell2mat(arrayfun(@(h)get_param(h,'Position'),q(:),'UniformOutput',false)); [~,j]=sort(xy(:,2)); q=q(j);
end
function wire(m,a,b)
% SPS conserving connections; adding branches from an existing port is supported.
try, add_line(m,a,b,'autorouting','on'); return; catch ME, first=ME; end
try, add_line(m,b,a,'autorouting','on'); return; catch, end
% Fallback for a connected electrical port: branch from a point on its line.
for swap=0:1
    if swap, aa=b; bb=a; else, aa=a; bb=b; end
    try
        lh=get_param(aa,'Line'); if lh<0, continue; end
        pts=get_param(lh,'Points'); dst=get_param(bb,'Position');
        v=pts(2:end,:)-pts(1:end-1,:); [~,j]=max(sum(abs(v),2));
        mid=round((pts(j,:)+pts(j+1,:))/2);
        elbow=[dst(1) mid(2)]; route=[mid;elbow;dst]; route=route([true;any(diff(route)~=0,2)],:);
        add_line(m,route); return;
    catch, end
end
rethrow(first);
end
function sig(m,a,ai,b,bi)
x=get_param(a,'PortHandles'); y=get_param(b,'PortHandles'); add_line(m,x.Outport(ai),y.Inport(bi),'autorouting','on');
end
function terminate(m,a,i,name,pos)
b=put('simulink/Sinks/Terminator',m,name,pos); sig(m,a,i,b,1);
end
function logSignal(m,a,i,name,p,x,y)
b=put('simulink/Sinks/To Workspace',m,['Log_' name],[x y x+135 y+27]);
set_param(b,'VariableName',name,'SaveFormat','Timeseries','Decimation',num(p.logDecimation),'MaxDataPoints','inf'); sig(m,a,i,b,1);
end
function zeroLog(m,name,p,x,y)
b=put('simulink/Sources/Constant',m,['Zero_' name],[x-75 y x-30 y+25]); set_param(b,'Value','0','SampleTime',num(p.dt)); logSignal(m,b,1,name,p,x,y);
end
function s=num(v), s=sprintf('%.17g',v); end
