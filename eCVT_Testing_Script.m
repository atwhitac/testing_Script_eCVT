% eCVT Testing Script

close all
%% Specify Data Location
time           = column(1);
time = (round((time-time(1))/10000))/100;
engineEngaged  = column(2);         
engineState    = column(3);  
engineSpeed    = column(4);            
enginePID      = column(5);  
engineP        = column(6);
engineI        = column(7);
engineD        = column(8);
primaryState   = column(9);
primaryEnc     = column(10);
primaryLC      = column(11);
primaryPID     = column(12);
secondaryState = column(13);
secondaryEnc   = column(14);
secondaryLC    = column(15);
secondaryPID   = column(16);
engineSpeedSet = 3500*ones(length(time), 1);

secondarymaxLC = max(abs(secondaryLC))
primarymaxLC = max(abs(primaryLC))

%% Engine Loop
engineLoopFig = figure('Name', 'Engine Loop Data', 'NumberTitle', 'off');
engineError = engineSpeedSet-engineSpeed;
engineData = array2table([time engineSpeed engineSpeedSet engineP engineI engineD enginePID engineError engineState]);
engineData = renamevars(engineData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6" "Var7" "Var8" "Var9"], ["Time" "Engine Speed Actual" "Engine Speed Setpoint" "Engine P" "Engine I" "Engine D" "Engine PID" "Engine Error" "Engine State"]);
engineVars = {'Engine State',{'Engine Speed Actual', 'Engine Speed Setpoint'}, 'Engine Error',{'Engine P', 'Engine I','Engine D','Engine PID'}};
engineLoopTL = stackedplot(engineData, engineVars, 'XVariable','Time');


%% Primary Loop
ratioPercentage = abs(floor(enginePID));
primaryEncTarget = lookup(ratioPercentage, 2);
primaryError = primaryEncTarget-primaryEnc;

primaryLoopFig = figure('Name', 'Primary Loop Data', 'NumberTitle', 'off');
primaryData = array2table([time primaryEnc primaryEncTarget primaryPID primaryError primaryState]);
primaryData = renamevars(primaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6"], ["Time" "Primary Encoder Ticks" "Primary Encoder Ticks Target" "Primary PID" "Primary Error" "Primary State"]);
primaryVars = {'Primary State', {'Primary Encoder Ticks', 'Primary Encoder Ticks Target'}, 'Primary Error', 'Primary PID'};
primaryLoopTL = stackedplot(primaryData, primaryVars, 'XVariable','Time');


%% Secondary Loop
secondaryEncTarget = lookup(ratioPercentage, 3);
secondaryError = secondaryEncTarget-secondaryEnc;

secondaryLoopFig = figure('Name', 'Secondary Loop Data', 'NumberTitle', 'off');
secondaryData = array2table([time secondaryEnc secondaryEncTarget secondaryPID secondaryError secondaryState]);
secondaryData = renamevars(secondaryData, ["Var1" "Var2" "Var3" "Var4" "Var5" "Var6"], ["Time" "Secondary Encoder Ticks" "Secondary Encoder Ticks Target" "Secondary PID" "Secondary Error" "Secondary State"]);
secondaryVars = {'Secondary State', {'Secondary Encoder Ticks', 'Secondary Encoder Ticks Target'}, 'Secondary Error', 'Secondary PID'};
secondaryLoopTL = stackedplot(secondaryData, secondaryVars, 'XVariable','Time');




%% Secondary Force Loop

secondaryForceLoopFig = figure('Name', 'Secondary Load Cell Force', 'NumberTitle', 'off');
plot(time, secondaryLC);




%% User Defined Functions
function [x] = column(n)

testData = 'ben hill.csv';  %<--- This is where input .csv goes

T = table2array(readtable(testData, 'NumHeaderLines',0));
if n <= 0
    x = zeros(length(T(:,1)),1);
else
    x = T(:,n);
end
end

function [x] = lookup(n1,n2) % Lookup Table Function
lookuptable = table2array(readtable('eCVT Encoder Lookup Table.csv','NumHeaderLines',1)); %<--- Lookup table .csv
x = lookuptable(101-n1,n2);

end