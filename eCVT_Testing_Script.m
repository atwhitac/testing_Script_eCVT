%% eCVT Testing Script


testData = uigetfile('*.csv');  %<-Opens window to select file to be tested, make sure file is in MATLAB directory!!



%% Global Variables

global lookuptable;
lookuptable = table2array(readtable('eCVT Encoder Lookup Table.csv','NumHeaderLines',1)); %<--- Lookup table .csv
global clampingForceFOS;
clampingForceFOS = 0.5;
global highRatio;
highRatio = 0.857; %0 Ratio Percentage
global lowRatio;
lowRatio = 4.496; %100 Ratio Percentage

close all
%% Constants


%% Specify Data Location
time               = column(1, testData);
time               = (round((time-time(1))/10000))/100;
sSpeed             = column(2, testData)*6.95;
carSpeed           = column(2, testData)*3.1415*1.916/60;

% Engine
eState             = column(3, testData);  
eSpeed             = column(4, testData);            
ePID               = column(5, testData);  
eP                 = column(6, testData); 
eI                 = column(7, testData);
eD                 = column(8, testData);

% Primary
pState             = column(9, testData);
pEncoder           = column(10, testData);
pClampForce        = column(11, testData)*1.8333;
pMotorCurrent      = column(12, testData)/43.4;
pControllerOutput  = column(13, testData);

%Secondary
sState             = column(14, testData);
sEncoder           = column(15, testData);
sClampForce        = column(16, testData)*-1.8333;
sMotorCurrent      = column(17, testData)/43.4;
sControllerOutput  = column(18, testData);
sEncoderPID        = column(19, testData);
sLoadCellPID       = column(20, testData);
sLoadCellP         = column(21, testData);
sLoadCellI         = column(22, testData);
sLoadCellD         = column(23, testData);

i = 1;

slipRatio = zeros(length(time),1);

while(i<length(time))
    if(eState(i) > 1)
       slipRatio(i) = (sSpeed(i) ./ (eSpeed(i)./pEncoderToRatio(pEncoder(i)))) - 1;
    end
    i = i + 1;
end



carDistance = cumtrapz(carSpeed)/100; % ft

eSetPoint          = 3500 * ones(length(time), 1);
PIDZeroMark        = zeros(length(time), 1);

ratioPercentage    = abs(floor(ePID));
pEncoderTarget     = ratioPercentageToPEncoder(ratioPercentage);
sEncoderTarget     = ratioPercentageToSEncoder(ratioPercentage);
clampForceTarget   = ratioPercentageToClampForce(ratioPercentage);

secondaryMaxClamp  = max(abs(sClampForce))
primaryMaxClamp    = max(abs(pClampForce))
engineSpeedMax     = max(abs(eSpeed))

primaryAverageDraw = mean(pMotorCurrent)%(interp1(time,1:length(time),13,'nearest'):interp1(time,1:length(time),18,'nearest')))
secondaryAverageDraw = mean(sMotorCurrent)%(interp1(time,1:length(time),13,'nearest'):interp1(time,1:length(time),18,'nearest')))
ratio = primaryAverageDraw/secondaryAverageDraw
AverageRPM = mean(eSpeed)%(interp1(time,1:length(time),2,'nearest'):interp1(time,1:length(time),17,'nearest')))


%% Engine Loop
engineLoopFig = figure('Name', 'Engine Loop Data', 'NumberTitle', 'off');
engineData = array2table([time eSpeed eSetPoint eP eI eD ePID eState carSpeed carDistance]);
engineData = renamevars(engineData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9" "Var10"], ["Time" "Engine Speed" "Engine Speed Setpoint" "Engine P" "Engine I" "Engine D" "Engine PID" "Engine State" "Car Speed" "Car Distance"]);
engineVars = {'Engine State',{'Engine Speed', 'Engine Speed Setpoint'}, {'Engine P', 'Engine I','Engine D','Engine PID'}, 'Car Speed', 'Car Distance'};
engineLoopTL = stackedplot(engineData, engineVars, 'XVariable','Time');
engineLoopTL.AxesProperties(2).YLimits = [3000 4000];
engineLoopTL.AxesProperties(3).YLimits = [0 100];
engineLoopTL.AxesProperties(4).YLimits = [0 40];


%% Primary Loop


primaryLoopFig = figure('Name', 'Primary Loop Data', 'NumberTitle', 'off');
primaryData = array2table([time pEncoder pEncoderTarget pControllerOutput PIDZeroMark pState pClampForce clampForceTarget pMotorCurrent]);
primaryData = renamevars(primaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9"], ["Time" "Primary Encoder Ticks" "Primary Encoder Ticks Target" "Primary PID" "PIDZeroMark" "Primary State" "Primary Force" "Target Force" "Motor Current"]);
primaryVars = {'Primary State', {'Primary Encoder Ticks', 'Primary Encoder Ticks Target'}, {'Primary PID', 'PIDZeroMark'}, {'Primary Force', 'Target Force'}, 'Motor Current'};
primaryLoopTL = stackedplot(primaryData, primaryVars, 'XVariable','Time');


%% Secondary Loop

secondaryLoopFig = figure('Name', 'Secondary Loop Data', 'NumberTitle', 'off');
secondaryData = array2table([time sEncoder sEncoderTarget sControllerOutput sEncoderPID sLoadCellPID sLoadCellP sLoadCellI sLoadCellD PIDZeroMark  sState sClampForce clampForceTarget sMotorCurrent slipRatio]);
secondaryData = renamevars(secondaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9" "Var10" "Var11" "Var12" "Var13" "Var14" "Var15"], ["Time" "Secondary Encoder Ticks" "Secondary Encoder Ticks Target" "Secondary Controller Output" "sEncoderPID" "sLoadCellPID" "sLoadCellP" "sLoadCellI" "sLoadCellD" "PIDZeroMark" "Secondary State" "Secondary Force" "Target Force" "Motor Current" "Unfiltered Slip Ratio"]);
secondaryVars = {'Secondary State', {'Secondary Encoder Ticks', 'Secondary Encoder Ticks Target'}, {'Secondary Controller Output', 'sEncoderPID', 'sLoadCellPID', 'PIDZeroMark'}, {'sLoadCellP', 'sLoadCellI', 'sLoadCellD', 'PIDZeroMark'}, {'Secondary Force', 'Target Force'}, 'Motor Current', 'Unfiltered Slip Ratio'};
secondaryLoopTL = stackedplot(secondaryData, secondaryVars, 'XVariable','Time');
secondaryLoopTL.AxesProperties(7).YLimits = [-0.4 0.1];


%% Load Cell Forces

ForceFig = figure('Name', 'Load Cell Forces', 'NumberTitle', 'off');
plot(time, pClampForce);
hold on;
plot(time, -sClampForce);
hold off;
legend("Primary", "Secondary");

%% Load Cell Forces

%ForceFigFiltered = figure('Name', ['Load Cell Force Filtered'], 'NumberTitle', 'off');
%plot(time, filter([1000, 1000],1,transpose(primaryLC+secondaryLC)));


%% User Defined Functions
function [x] = column(n, tData)

T = table2array(readtable(tData, 'NumHeaderLines',0));
if n <= 0
    x = zeros(length(T(:,1)),1);
else
    x = T(:,n);
end
end

function [x] = ratioPercentageToSEncoder(ratioPercentage) % Lookup Table Function
global lookuptable;
if(ratioPercentage > 100)
    ratioPercentage = 100;
end
if(ratioPercentage < 0)
    ratioPercentage = 0;
end
x = lookuptable(101-ratioPercentage,3);

end

function [x] = ratioPercentageToPEncoder(ratioPercentage) % Lookup Table Function
global lookuptable;
if(ratioPercentage > 100)
    ratioPercentage = 100;
end
if(ratioPercentage < 0)
    ratioPercentage = 0;
end
x = lookuptable(101-ratioPercentage,2);

end

function ratio = pEncoderToRatio(pEncoder) % Lookup Table Function
global lookuptable;
global highRatio;
global lowRatio;
i = 1;
if(pEncoder < 8010)
    ratio = lowRatio;
elseif(pEncoder > 48295)
       ratio = highRatio;
else
    while(i<102)
        if(lookuptable(i,2)>pEncoder)
            break
        end
        i = i + 1;
    end

    pEncoder1 = lookuptable(i-1,2);
    pEncoder2 = lookuptable(i,2);

    ratioPercent1 = lookuptable(i-1,1);
    ratioPercent2 = lookuptable(i,1);

    ratioPercent = ratioPercent1 + (ratioPercent2-ratioPercent1) * ( (pEncoder - pEncoder1) / (pEncoder2-pEncoder1) );

    ratio = lowRatio - ((lowRatio-highRatio)*(1-ratioPercent*0.01));

    end
end

function [x] = ratioPercentageToClampForce(ratioPercentage) % Lookup Table Function
global lookuptable;
global clampingForceFOS;
x = lookuptable(101-ratioPercentage,4) * clampingForceFOS;

end


function [dist] = getDistanceSpeed(rearWheelSpeed)
dist = zeros(length(rearWheelSpeed));


end
