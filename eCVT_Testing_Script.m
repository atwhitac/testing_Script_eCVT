%% eCVT Testing Script


testData = uigetfile('*.csv');  %<-Opens window to select file to be tested, make sure file is in MATLAB directory!!

%% Global Variables

global lookuptable;
lookuptable = table2array(readtable('eCVT Encoder Lookup Table.csv','NumHeaderLines',1)); %<--- Lookup table .csv
global clampingForceFOS;
clampingForceFOS = 0.8;
global highRatio;
highRatio = 0.857; %0 Ratio Percentage
global lowRatio;
lowRatio = 4.496; %100 Ratio Percentage

close all
%% Constants

d1 = designfilt("lowpassiir",FilterOrder=2, ...
    HalfPowerFrequency=0.05,DesignMethod="butter");

%% Specify Data Location
time               = column(1, testData);
time               = (round((time-time(1))/10000))/100;
carSpeed           = column(2, testData)*5.759/60; % (rot / min) * (5.759 ft/rot) * (1 min / 60 sec) = ft / s
filteredCarSpeed   = zeros(length(carSpeed), 1);
i = 7;
averageAccel = mean(gradient(carSpeed));
while(i<length(carSpeed))
lastAverage = (filteredCarSpeed(i-6) + filteredCarSpeed(i-5) + filteredCarSpeed(i-4) + filteredCarSpeed(i-3) + filteredCarSpeed(i-2) + filteredCarSpeed(i-1))/6;

    if((carSpeed(i) > lastAverage*1.5 || carSpeed(i) < lastAverage*0.7) && lastAverage>2)
        filteredCarSpeed(i) = filteredCarSpeed(i-1);
    else
        filteredCarSpeed(i) = carSpeed(i);
    end
    i=i+1;
end
filteredCarSpeed = filtfilt(d1,filteredCarSpeed);
carAccel = gradient(filteredCarSpeed);
sSpeed           = column(2, testData)*6.95;

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
pSpeed             = gradient(pEncoder)*100/128*60; %RPM
pClampForce        = column(11, testData)*1.8333;
pMotorCurrent      = column(12, testData)/3.1 - 4.5;
pControllerOutput  = column(13, testData);

%Secondary
sState             = column(14, testData);
sEncoder           = column(15, testData);
sMotSpeed             = gradient(sEncoder)*100/128*60; %RPM
sClampForce        = column(16, testData)*-1.8333;
sMotorCurrent      = column(17, testData)/3.1 - 4.5;
sControllerOutput  = column(18, testData);
sEncoderPID        = column(19, testData);
sLoadCellPID       = column(20, testData);
sLoadCellP         = column(21, testData);
sLoadCellI         = column(22, testData);
sLoadCellD         = column(23, testData);
fBrakePressure         = column(24, testData);
rBrakePressure         = column(25, testData);


timesTriggered = zeros(length(time),1);
Strikes = 0;
maxNumTriggered = 0;
totalTriggers = 0;

pEncoderLast = 0;
sEncoderLast = 0;
pClampForceLast = 0;
sClampForceLast = 0;

resetTrigger = 1;

i = 1;
while(i < length(time)-1)
    pEncoderDiff = abs(pEncoder(i+1) - pEncoderLast);
    sEncoderDiff = abs(sEncoder(i+1) - sEncoderLast);
    pForceDiff = abs(pClampForce(i+1) - pClampForceLast);
    sForceDiff = abs(sClampForce(i+1) - sClampForceLast);
    if(abs(pControllerOutput(i)) > 35 && (pEncoderDiff <= 4 || pForceDiff <= 2))
        Strikes = Strikes + 1;
        if(Strikes > 50 && resetTrigger == 1)
            totalTriggers = totalTriggers + 1;
            timesTriggered(totalTriggers) = time(i);
            resetTrigger = 0;
        end
    else
        pEncoderLast = pEncoder(i);
        sEncoderLast = sEncoder(i);
        pClampForceLast = pClampForce(i);
        sClampForceLast = sClampForce(i);
        if (Strikes > maxNumTriggered)
            maxnumi = time(i);
            maxNumTriggered = Strikes;
        end
        Strikes = 0;
        resetTrigger = 1;
    end

    i = i + 1;
end



i = 1;

slipRatio = zeros(length(time),1);
Ratio = zeros(length(time),1);
while(i<length(time))
    if(eState(i) > 1)
        Ratio(i) = pEncoderToRatio(pEncoder(i));
        slipRatio(i) = 1 - (sSpeed(i) ./ (eSpeed(i)./Ratio(i)));

    end
    i = i + 1;
end




carDistance = cumtrapz(filteredCarSpeed)/100; % ft

eSetPoint          = 3500 * ones(length(time), 1);
PIDZeroMark        = zeros(length(time), 1);
MotorMaxSpeedMark  = ones(length(time),1)*7120;

ratioPercentage    = abs(floor(ePID));
pEncoderTarget     = ratioPercentageToPEncoder(ratioPercentage);
sEncoderTarget     = ratioPercentageToSEncoder(ratioPercentage);

i = 1;
while(i < length(time))
    if(eState(i) == 1)
        pEncoderTarget(i) = 4004 - 1000;
        sEncoderTarget(i) = 35300/2;
    end
    i = i + 1;
end
clampForceTarget   = ratioPercentageToClampForce(ratioPercentage);

secondaryMaxClamp  = max(abs(sClampForce))
primaryMaxClamp    = max(abs(pClampForce))
engineSpeedMax     = max(abs(eSpeed))

FirstTime = 20;
SecondTime = 40;
i = 1;
while(FirstTime == 0 && i < length(time))
    if(filteredCarSpeed(i) > 10)
         FirstTime = time(i);
    end
    i = i + 1;
end
j=1;
while(SecondTime == 0 && j<length(time))
    if(carDistance(j) > carDistance(i)+100)
         SecondTime = time(j);
    end
    j = j + 1;
end

i = 1;
while(i<length(time))
try
    lookuptable(1001-ePID(i),2);
catch
    fucked = i
end
i = i + 1;
end
FirstTime = 141.6;
SecondTime = 145.6;

AccelTime = SecondTime - FirstTime
primaryAverageDraw = mean(pMotorCurrent(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
secondaryAverageDraw = mean(sMotorCurrent(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
AverageRPM = mean(eSpeed(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
StandardDeviation = std(eSpeed(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
AverageSlip = mean(slipRatio(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
SError = sEncoderTarget - sEncoder;
AverageSecError = mean(SError(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
Distance = carDistance(SecondTime*100) - carDistance(FirstTime*100)

%% Engine Loop
engineLoopFig = figure('Name', 'Engine Loop Data', 'NumberTitle', 'off');
engineData = array2table([time eSpeed eSetPoint eP eI eD ePID eState carSpeed filteredCarSpeed fBrakePressure, rBrakePressure]);
engineData = renamevars(engineData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9" "Var10" "Var11" "Var12"], ["Time" "Engine Speed" "Engine Speed Setpoint" "Engine P" "Engine I" "Engine D" "Engine PID" "Engine State" "Car Speed" "Filtered Car Speed" "Front Brake" "Rear Brake"]);
engineVars = {'Engine State',{'Engine Speed', 'Engine Speed Setpoint'}, {'Engine P', 'Engine I','Engine D','Engine PID'}, {'Filtered Car Speed' 'Car Speed'}, {'Front Brake', 'Rear Brake'}};
engineLoopTL = stackedplot(engineData, engineVars, 'XVariable','Time');
engineLoopTL.AxesProperties(2).YLimits = [-100 3800];
engineLoopTL.AxesProperties(4).YLimits = [0 60];


%% For design presentation


engineLoopFig = figure('Name', 'Engine Loop Data', 'NumberTitle', 'off');
engineData = array2table([time eSpeed eSetPoint filteredCarSpeed]);
engineData = renamevars(engineData, ["Var1" "Var2" "Var3" "Var4"], ["Time" "Engine Speed (RPM)" " " "Car Speed (ft/s)"]);
engineVars = {{'Engine Speed (RPM)', ' '}, 'Car Speed (ft/s)'};
engineLoopTL = stackedplot(engineData, engineVars, 'XVariable','Time');
engineLoopTL.AxesProperties(1).YLimits = [3000 4000];
engineLoopTL.AxesProperties(2).YLimits = [0 50];
engineLoopTL.LineWidth = 1.5;
engineLoopTL.FontSize = 16;

%% Primary Loop


primaryLoopFig = figure('Name', 'Primary Loop Data', 'NumberTitle', 'off');
primaryData = array2table([time pEncoder pEncoderTarget pControllerOutput PIDZeroMark pState pClampForce clampForceTarget pSpeed MotorMaxSpeedMark pMotorCurrent]);
primaryData = renamevars(primaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9" "Var10" "Var11"], ["Time" "Primary Encoder Ticks" "Primary Encoder Ticks Target" "Primary PID" "PIDZeroMark" "Primary State" "Primary Force" "Target Force" "Primary Speed" "Max Motor Speed" "Motor Current"]);
primaryVars = {'Primary State', {'Primary Encoder Ticks', 'Primary Encoder Ticks Target'}, {'Primary PID', 'PIDZeroMark'}, {'Primary Speed', 'Max Motor Speed'}, {'Primary Force', 'Target Force'}, 'Motor Current'};
primaryLoopTL = stackedplot(primaryData, primaryVars, 'XVariable','Time');


%% Secondary Loop

secondaryLoopFig = figure('Name', 'Secondary Loop Data', 'NumberTitle', 'off');
secondaryData = array2table([time sEncoder sEncoderTarget sControllerOutput sEncoderPID sLoadCellPID sLoadCellP sLoadCellI sLoadCellD PIDZeroMark  sState sClampForce clampForceTarget sMotorCurrent slipRatio]);
secondaryData = renamevars(secondaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9" "Var10" "Var11" "Var12" "Var13" "Var14" "Var15"], ["Time" "Secondary Encoder Ticks" "Secondary Encoder Ticks Target" "Secondary Controller Output" "sEncoderPID" "sLoadCellPID" "sLoadCellP" "sLoadCellI" "sLoadCellD" "PIDZeroMark" "Secondary State" "Secondary Force" "Target Force" "Motor Current" "Unfiltered Slip Ratio"]);
secondaryVars = {'Secondary State', {'Secondary Encoder Ticks', 'Secondary Encoder Ticks Target'}, {'Secondary Controller Output', 'sEncoderPID', 'sLoadCellPID', 'PIDZeroMark'}, {'sLoadCellP', 'sLoadCellI', 'sLoadCellD', 'PIDZeroMark'}, {'Secondary Force', 'Target Force'}, 'Motor Current', 'Unfiltered Slip Ratio'};
secondaryLoopTL = stackedplot(secondaryData, secondaryVars, 'XVariable','Time');
secondaryLoopTL.AxesProperties(7).YLimits = [-.2 .2];

%% Secondary Loop For Presentation

secondaryLoopFig = figure('Name', 'Secondary Loop Data', 'NumberTitle', 'off');
secondaryData = array2table([time sEncoder sEncoderTarget sClampForce clampForceTarget sControllerOutput]);
secondaryData = renamevars(secondaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6"], ["Time" "Secondary Position" "Secondary Target Position" "Secondary Force" "Secondary Target Force" "Secondary Controller Output"]);
secondaryVars = {{'Secondary Position', 'Secondary Target Position'}, {'Secondary Force', 'Secondary Target Force'}, 'Secondary Controller Output'};
secondaryLoopTL = stackedplot(secondaryData, secondaryVars, 'XVariable','Time');
secondaryLoopTL.AxesProperties(1).YLimits = [5000 20000];
secondaryLoopTL.AxesProperties(2).YLimits = [150 450];
secondaryLoopTL.AxesProperties(3).YLimits = [-140 140];
secondaryLoopTL.LineWidth = 1.5;
secondaryLoopTL.FontSize = 16;

%% Primary Loop For Presentation

primaryLoopFig = figure('Name', 'Primary Loop Data', 'NumberTitle', 'off');
primaryData = array2table([time pEncoder pEncoderTarget pClampForce clampForceTarget pControllerOutput]);
primaryData = renamevars(primaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6"], ["Time" "Primary Position" "Primary Target Position" "Primary Force" "Primary Target Force" "Primary Controller Output"]);
primaryVars = {{'Primary Position', 'Primary Target Position'}, {'Primary Force', 'Primary Target Force'}, 'Primary Controller Output'};
primaryLoopTL = stackedplot(primaryData, primaryVars, 'XVariable','Time');
primaryLoopTL.AxesProperties(1).YLimits = [0 23000];
%primaryLoopTL.AxesProperties(2).YLimits = [150 450];
%primaryLoopTL.AxesProperties(3).YLimits = [-140 140];
primaryLoopTL.LineWidth = 1.5;
primaryLoopTL.FontSize = 16;

%% Primary and Secondary Aggergate

primarySecondarayFig = figure('Name', 'Primary + Secondary Data (Signs reversed on secondary)', 'NumberTitle', 'off');
primarySecondaryData = array2table([time pControllerOutput -sControllerOutput pSpeed -sMotSpeed pClampForce sClampForce]);
primarySecondaryData = renamevars(primarySecondaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7"], ["Time" "Primary PID Output" "Secondary PID Output" "Primary Motor Speed" "Secondary Motor Speed" "Primary Clamp Force" "Seconary Clamp Force"]);
primarySecondaryVars = {{'Primary PID Output', 'Secondary PID Output'}, {'Primary Motor Speed', 'Secondary Motor Speed'}, {'Primary Clamp Force', 'Seconary Clamp Force'}};
primarySecondaryTL = stackedplot(primarySecondaryData, primarySecondaryVars, 'XVariable','Time');


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
x(1) = [];

end
end

function [x] = ratioPercentageToSEncoder(ratioPercentage) % Lookup Table Function
global lookuptable;
if(ratioPercentage > 1000)
    ratioPercentage = 1000;
end
if(ratioPercentage < 0)
    ratioPercentage = 0;
end

try
x = lookuptable(1001-ratioPercentage,3);
catch
x = 0;
end
end

function [x] = ratioPercentageToPEncoder(ratioPercentage) % Lookup Table Function
global lookuptable;
if(ratioPercentage > 1000)
    ratioPercentage = 1000;
end
if(ratioPercentage < 0)
    ratioPercentage = 0;
end
try
    x = lookuptable(1001-ratioPercentage,2);
catch
    x = 0;
end

end

function ratio = pEncoderToRatio(pEncoder) % Lookup Table Function
global lookuptable;
global highRatio;
global lowRatio;
i = 1;
if(pEncoder < 4004)
    ratio = lowRatio;
elseif(pEncoder > 24148)
       ratio = highRatio;
else
    while(i<1001)
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

    ratio = 1 ./ sqrt((1/highRatio^2) - ratioPercent .* ((1/(highRatio^2) - 1/(lowRatio^2)) / (1001-1)));

    end
end

function [x] = ratioPercentageToClampForce(ratioPercentage) % Lookup Table Function
global lookuptable;
global clampingForceFOS;

try
x = lookuptable(1001-ratioPercentage,4) * clampingForceFOS;
catch
x = 0;
end
end


function [dist] = getDistanceSpeed(rearWheelSpeed)
dist = zeros(length(rearWheelSpeed));


end
