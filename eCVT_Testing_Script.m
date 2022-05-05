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

d1 = designfilt("lowpassiir",FilterOrder=2, ...
    HalfPowerFrequency=0.05,DesignMethod="butter");

%% Specify Data Location
time               = column(1, testData);
time               = (round((time-time(1))/10000))/100;
carSpeed           = column(2, testData)*0.047995;
filteredCarSpeed   = zeros(length(carSpeed), 1);
i = 7;
averageAccel = mean(gradient(carSpeed));
while(i<length(carSpeed))
lastAverage = (carSpeed(i-6) + carSpeed(i-5) + carSpeed(i-4) + carSpeed(i-3) + carSpeed(i-2) + carSpeed(i-1))/6;

    if((carSpeed(i) > lastAverage*1.5 || carSpeed(i) < lastAverage*0.5) && lastAverage>2)
        filteredCarSpeed(i) = filteredCarSpeed(i-1);
    else
        filteredCarSpeed(i) = carSpeed(i);
    end
    i=i+1;
end
filter0edCarSpeed = filtfilt(d1,filteredCarSpeed);
carAccel = gradient(filteredCarSpeed);
sSpeed           = filteredCarSpeed;

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
pMotorCurrent      = column(12, testData)/43.4;
pControllerOutput  = column(13, testData);

%Secondary
sState             = column(14, testData);
sEncoder           = column(15, testData);
sMotSpeed             = gradient(sEncoder)*100/128*60; %RPM
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
        pEncoderTarget(i) = 5009;
        sEncoderTarget(i) = 35300;
    end
    i = i + 1;
end
clampForceTarget   = ratioPercentageToClampForce(ratioPercentage);

secondaryMaxClamp  = max(abs(sClampForce))
primaryMaxClamp    = max(abs(pClampForce))
engineSpeedMax     = max(abs(eSpeed))

FirstTime = 0;
SecondTime = 0;
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
    lookuptable(1000-ePID(i),2);
catch
    fucked = i
end
i = i + 1;
end
FirstTime = 75;
SecondTime = 440;

AccelTime = SecondTime - FirstTime
primaryAverageDraw = mean(pMotorCurrent(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
secondaryAverageDraw = mean(sMotorCurrent(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
AverageRPM = mean(eSpeed(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
StandardDeviation = std(eSpeed(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))
AverageSlip = mean(slipRatio(interp1(time,1:length(time),FirstTime,'nearest'):interp1(time,1:length(time),SecondTime,'nearest')))

pEncoderToRatio(12000)
%% Engine Loop
engineLoopFig = figure('Name', 'Engine Loop Data', 'NumberTitle', 'off');
engineData = array2table([time eSpeed eSetPoint eP eI eD ePID eState carSpeed filteredCarSpeed carDistance carAccel]);
engineData = renamevars(engineData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9" "Var10" "Var11" "Var12"], ["Time" "Engine Speed" "Engine Speed Setpoint" "Engine P" "Engine I" "Engine D" "Engine PID" "Engine State" "Car Speed" "Filtered Car Speed" "Car Distance" "Car Acceleration"]);
engineVars = {'Engine State',{'Engine Speed', 'Engine Speed Setpoint'}, {'Engine P', 'Engine I','Engine D','Engine PID'}, {'Filtered Car Speed' 'Car Speed'}, 'Car Distance', 'Car Acceleration'};
engineLoopTL = stackedplot(engineData, engineVars, 'XVariable','Time');
engineLoopTL.AxesProperties(2).YLimits = [2400 3800];
%engineLoopTL.AxesProperties(3).YLimits = [-25 125];
engineLoopTL.AxesProperties(4).YLimits = [0 40];


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
secondaryLoopTL.AxesProperties(7).YLimits = [-0.4 0.4];


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
if(ratioPercentage > 999)
    ratioPercentage = 999;
end
if(ratioPercentage < 0)
    ratioPercentage = 0;
end

try
x = lookuptable(1000-ratioPercentage,3);
catch
x = 0;
end
end

function [x] = ratioPercentageToPEncoder(ratioPercentage) % Lookup Table Function
global lookuptable;
if(ratioPercentage > 999)
    ratioPercentage = 999;
end
if(ratioPercentage < 0)
    ratioPercentage = 0;
end
try
    x = lookuptable(1000-ratioPercentage,2);
catch
    x = 0;
end

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

    ratio = 1 ./ sqrt((1/highRatio^2) - ratioPercent .* ((1/(highRatio^2) - 1/(lowRatio^2)) / (1000-1)));

    end
end

function [x] = ratioPercentageToClampForce(ratioPercentage) % Lookup Table Function
global lookuptable;
global clampingForceFOS;

try
x = lookuptable(1000-ratioPercentage,4) * clampingForceFOS;
catch
x = 0;
end
end


function [dist] = getDistanceSpeed(rearWheelSpeed)
dist = zeros(length(rearWheelSpeed));


end
