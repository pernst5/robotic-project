% ENSC192 - Spring 2021 - Section 4 - Preston Ernst
% Assignment: A: Problem
% Date: 04/14/2021
% Collaborators: N/A
% Find: Scan and find object, go and hit object, scan for exit, turn and
% leave arena
% Given: Use a functions for servo and motor control
% Constants: Arduino pins
% Variables: speed,angle,distance for robot control
% Assume: Pin configuration is constant and on COM3
%------------- BEGIN CODE --------------
clc; clear; clear Uno;
Uno = arduino('COM3','Uno','Libraries',{'Ultrasonic','Servo','I2C'}); % declare the arduino object and set on COM3
% include libraries for motor control

%%
% Assign and Configure Pins for Motor Control
enL = 'D5'; in1 = 'D4'; in2 = 'D2'; % Motor A (LEFT) pins
enR = 'D6'; in3 = 'D8'; in4 = 'D7'; % Motor B (RIGHT) pins
% Configure motor L (Left)
configurePin(Uno,enL,'PWM'); 
configurePin(Uno,in1,'DigitalOutput'); 
configurePin(Uno,in2,'DigitalOutput'); 
% Configure motor R (Right)
configurePin(Uno,enR,'PWM'); 
configurePin(Uno,in4,'DigitalOutput'); 
configurePin(Uno,in3,'DigitalOutput'); 

% initialize MPU sensor
imu = mpu6050(Uno);
gyroReading = zeros(200,3); % Preallocate array for gyro data

% Assign and configure pins for servo control
    frontServoPin = 'D10'; % variable use to assign specific digital pin
    configurePin(Uno,frontServoPin,'Servo'); % servo control

% Set Servo Characteristics - Pulse Duration Values - servo dependent.
    rotMinPulse = 500*10^-6; % minimum pulse width, rotate left position
    rotMaxPulse = 2400*10^-6; % maximum pulse width, rotate right position
    
% Servo Setup: Servo pins and pulse characteristics assigned to device
    frontScanServo = servo(Uno,frontServoPin,'MinPulseDuration',rotMinPulse,...
                                'MaxPulseDuration', rotMaxPulse);
              
% Configure pins for ultrasonic sensor
    usEchoPin = 'A1'; % pin connected to ultrasonic echo
        configurePin(Uno,usEchoPin,'Ultrasonic'); % Ultrasonic echo
    usTriggerPin = 'A0'; % pin connected to ultrasonic trigger
        configurePin(Uno,usTriggerPin,'Ultrasonic'); % Ultrasonic trigger   
    
% Assign Ultrasonic sensor to Arduino pins for trigger and echo
%   - Output format switched to double to simplify calculations and display
    frontUSsensor = ultrasonic(Uno,usTriggerPin,usEchoPin,'OutputFormat','double');

 % Speaker setup
    tonePin1 = 'D11'; configurePin(Uno, tonePin1, 'Tone');
    
 % ButtonWait Setup
    ledPin = 'D12'; configurePin(Uno,ledPin,'DigitalOutput'); % LED control
    buttonPin = 'D13'; configurePin(Uno,buttonPin,'DigitalInput'); % push-button input
    

    ButtonWait(Uno, ledPin, buttonPin, 0.1); % fast flash
    pause(2);
  %%
   
% Complete one servo sweap and place scan data in TargetScanData matrix
TargetScanData = ServoSweap(0.01,1,frontScanServo,frontUSsensor);

table(TargetScanData(:,1),TargetScanData(:,2),'VariableNames',{'Angle(degrees)','Distance(m)'}) % Output a table of all of the variables 
% Plot the results
subplot(1,2,1)
plot(TargetScanData(:,1),TargetScanData(:,2),'ro', 'MarkerSize', 2, ...
     'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r')
title("Returned Range for Object Scan");
xlabel("Angle - degrees"); ylabel("Range - m");

subplot(1,2,2)
polarplot((TargetScanData(:,1))/180 * pi + pi/2,TargetScanData(:,2),'o', 'MarkerSize', 2, ...
     'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g')
title("Polar Scan Plot");


% remove outliers
TargetScanData(1:5,2) = 0.5;
% for i = 1:length(TargetScanData)-5
%    x(i) = TargetScanData(i,2) - mean(TargetScanData(i:i+5,2))
% end


% identify the target angle
j = 1;
distanceDiff = zeros(length(TargetScanData)-1);
for i = 1:length(TargetScanData)-1 % For every data point in TargetScanData
distanceDiff(i) = (abs(TargetScanData(i+1,2) - TargetScanData(i,2)) / 2) ; % compute the difference between one pt. and the next pt.
if distanceDiff(i) > .04 % If this distance difference is larger than some range store this pt. as a possible point of the target
    targetPossions(j) = (i/(length(TargetScanData)-1));
    j = j + 1;
end
end
targetMiddlePos = (targetPossions(1)+targetPossions(end)) / 2; % The middle of the target is an average of all of the possible target possitions
writePosition (frontScanServo, targetMiddlePos); %Set the servo to the target position to check results.
fprintf("The target is %.1f degrees from the center\n",targetMiddlePos*180-90) %Out put result possition 


% calculate target distance
targetPts = (TargetScanData(:,2) < .45);
targetDist = mean(TargetScanData(targetPts,2)*2); % Compute the mean of all of the target distances
fprintf("The target distance is about %0.1f cm\n",targetDist*100) % Output the target distance away


desirAngle = targetMiddlePos*180 - 90; %Set the desired anlge you want to go to

% Call the function to turn to an angle, pause, and then move forward a distance
TurnAndDrive(2,desirAngle,targetDist*1.8,Uno,in1,in2,in3,in4,enL,enR,imu) 
% TurnAndDrive(speed,angle(-90 to 90),distance(m),Uno,in1,in2,in3,in4,enL,enR) 

    writeDigitalPin(Uno, ledPin, 1); pause(.5); % light LED
    writeDigitalPin(Uno, ledPin, 0); pause(.5); % dim LED
    playTone(Uno, tonePin1, 1318.51, .3); pause(0.02); 
    playTone(Uno, tonePin1, 1567.98, .3); pause(0.02);
    playTone(Uno, tonePin1, 2093.00, .3); 
    
%% Return to center and turn to face exit side
pause(1)
ReverseTurnAndDrive(2.2,-desirAngle,targetDist,Uno,in1,in2,in3,in4,enL,enR,imu) % Drive back to start possition then turn opposite of target angle
pause(0.5)
TurnAndDrive(2.2,180,0,Uno,in1,in2,in3,in4,enL,enR,imu) % turn 180 degrees and then don't drive forward
pause(.5)

%% Exit locator and Excape code
ExitScanData = ServoSweapExit(0.01,1,frontScanServo,frontUSsensor);

% remove outliers
ExitScanData(1:5,2) = 0.5;

table(ExitScanData(:,1),ExitScanData(:,2),'VariableNames',{'Angle(degrees)','Distance Exit(m)'}) % Output a table of all of the variables 
subplot(1,2,1)
plot(ExitScanData(:,1),ExitScanData(:,2),'ro', 'MarkerSize', 2, ...
     'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r')
title("Returned Range for Object Scan");
xlabel("Angle - degrees"); ylabel("Range - m");

subplot(1,2,2)
polarplot((ExitScanData(:,1))/180 * pi + pi/2,ExitScanData(:,2),'o', 'MarkerSize', 2, ...
     'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g')
title("Polar Scan Plot");

average = mean(ExitScanData(:,2));
meanIndexs = find(ExitScanData(:,2) >  (average + .1));
positions = [0:0.01:1];

sizeOfData = size(ExitScanData(:,2));
exitMiddlePos = mean(meanIndexs) / sizeOfData(1)
writePosition (frontScanServo, exitMiddlePos);
exitAngle = exitMiddlePos*180-90


% calculate exit distance
exitPts = (ExitScanData(:,2) < .45);
exitDist = max(ExitScanData)/100; % Compute the exit distances
fprintf("The exit distance is about %0.1f cm\n",exitDist(1)*100); % Output the exit distance away

% Call the function to turn to an angle, pause, and then move forward a distance
TurnAndDrive(2.2,exitAngle,exitDist*2.5,Uno,in1,in2,in3,in4,enL,enR,imu) 

disp("DONE!!!!!")
    playTone(Uno, tonePin1, 1318.51, 0.125); pause(0.13);
    playTone(Uno, tonePin1, 1567.98, 0.125); pause(0.13);
    playTone(Uno, tonePin1, 2637.02, 0.125); pause(0.13);
    playTone(Uno, tonePin1, 2093.00, 0.125); pause(0.13);
    playTone(Uno, tonePin1, 2349.32, 0.125); pause(0.13);
    playTone(Uno, tonePin1, 3135.96, 0.125); pause(0.13);
   
%%

function TargetScanData = ServoSweap (startPos,endPos,frontScanServo,frontUSsensor)
    % ServoSweap - Sweap servo 180 degrees and read distance at each point
    % Inputs: servo startPos and endPos the start and end possition
    % Outputs: N/A
    %
    % Author: Preston Ernst
    % Collaborators: N/A
    % Date: 04/14/2021; Last revision: 14-April-2021
    %------------- BEGIN CODE --------------
    writePosition (frontScanServo, startPos); 
    TargetScanData = zeros(100);
    i = 1; % create a variable i to be used in loop
    for position = startPos:0.01:endPos
        pause(0.0015);
        TargetScanData(i,1) = position*180 - 90; % Convert position to degrees
        % For each possition (left to right) complete the following
        writePosition (frontScanServo, position); % Set the servo to the possition
        pause(0.0015);
        TargetScanData(i,2) = readDistance(frontUSsensor); % Fill array distance with all distances read at each position
        tic
        while (TargetScanData(i,2) == Inf) % wait at each possition that returns an incorrect reading until you get a correct value
            TargetScanData(i,2) = readDistance(frontUSsensor);
            if toc > 1
               TargetScanData(i,2) = .5;
               break
            end
        end
        if TargetScanData(i,2) > 0.5 % If reading is outside range of area set reading to 0.5m
            TargetScanData(i,2) = 0.5;
        end
        i = i + 1; %Increment i by 1 each loop
    end
    %------------- END OF CODE --------------
end
function ButtonWait(a, l_Pin, b_Pin, d_Time)
    % Print status to Command Window
    fprintf("\nPRESS the breadboard button to continue. Waiting...\n");
    buttonSt = 0; % start with button state variable = 0
    while buttonSt == 0
        buttonSt = readDigitalPin(a,b_Pin); % check button state variable
        writeDigitalPin(a, l_Pin, 1); pause(d_Time); % light LED
        writeDigitalPin(a, l_Pin, 0); pause(d_Time); % dim LED
    end
    fprintf("\nButtonWait Over!\n");
    pause(1);
end  
function ExitScanData = ServoSweapExit (startPos,endPos,frontScanServo,frontUSsensor)
    % ServoSweap - Sweap servo 180 degrees and read distance at each point
    % Inputs: servo startPos and endPos the start and end possition
    % Outputs: N/A
    %
    % Author: Preston Ernst
    % Collaborators: N/A
    % Date: 04/14/2021; Last revision: 14-April-2021
    %------------- BEGIN CODE --------------
    writePosition (frontScanServo, startPos); 
    ExitScanData = zeros(100);
    i = 1; % create a variable i to be used in loop
    for position = startPos:0.01:endPos
        pause(0.0015);
        ExitScanData(i,1) = position*180 - 90; % Convert position to degrees
        % For each possition (left to right) complete the following
        writePosition (frontScanServo, position); % Set the servo to the possition
        pause(0.0015);
        ExitScanData(i,2) = readDistance(frontUSsensor); % Fill array distance with all distances read at each position
        tic
        while (ExitScanData(i,2) == Inf) % wait at each possition that returns an incorrect reading until you get a correct value
            ExitScanData(i,2) = readDistance(frontUSsensor);
            if toc > 1
               ExitScanData(i,2) = 2;
               break
            end
        end
        if ExitScanData(i,2) < 0.5 % If reading is outside range of area set reading to 0.5m
            ExitScanData(i,2) = 0.5;
        end
        i = i + 1; %Increment i by 1 each loop
    end
    %------------- END OF CODE --------------
end

function TurnAndDrive(speed,desirAngle,distance,Uno,in1,in2,in3,in4,enL,enR,imu)
    % TurnAndDrive - Turn to an inputed angle, then go an inputed distance
    % Inputs: input1 - speed(voltage of motors),angle(in
    % degrees,distance(m), Uno(arduino
    % object),in1,in2,in3,in4,enL,enR(defined pins
    % Outputs: N/A
    %
    % Author: Preston Ernst
    % Collaborators: N/A
    % Date: 04/14/2021; Last revision: 14-April-2021
    %------------- BEGIN CODE --------------
    i = 1;
    for go = 1:200
    tic

    yaw(1) = 0;
    if abs(yaw(i)-desirAngle) < 5  % If you are close to the desired shut off motors and leave loop
        fprintf('Stop  -----  Stop\n\n');
        writePWMVoltage(Uno,enL,0); % Set speed on L to zero
        writePWMVoltage(Uno,enR,0); % Set speed on R to zero
        break
    elseif yaw(i) < desirAngle
        fprintf('Backward -----  Forward\n');
        % Turn on Motor L Backward
        writeDigitalPin(Uno,in1,1); writeDigitalPin(Uno,in2,0); 
        writePWMVoltage(Uno,enL,2.2);  % Set left motor to input speed 

        % Turn on Motor R Forward
        writeDigitalPin(Uno,in4,0); writeDigitalPin(Uno,in3,1); 
        writePWMVoltage(Uno,enR,2.2); % Set right motor to input speed (adjusted to go straight)
    elseif yaw(i) > desirAngle
        % Turn Right
        fprintf('Forward  ----- Backward\n');
        % Turn on Motor L Forward
        writeDigitalPin(Uno,in1,0); writeDigitalPin(Uno,in2,1); 
        writePWMVoltage(Uno,enL,2.2);  % Set left motor to input speed 

        % Turn on Motor R Backward
        writeDigitalPin(Uno,in4,1); writeDigitalPin(Uno,in3,0); 
        writePWMVoltage(Uno,enR,2.2);
    end

    gyroReading(i,:) = 180/pi * readAngularVelocity(imu);   % Turn AngularVelocity(rad/s)data into (degrees/s)
    gyroReading(i,3) = (gyroReading(i,3)  - 0.6888); % Take out error in data reading
    yaw(i+1) = yaw(i) + gyroReading(i,3)*toc;        % yaw (degrees) from start equals (degrees/s) times elapsed time
    disp(yaw(i))
    i = i + 1;
    end
    
    pause(2)
    
    fprintf('Forward ----- Forward\n');
    % Turn on Motor L Forward
    writeDigitalPin(Uno,in1,0); writeDigitalPin(Uno,in2,1); 
    writePWMVoltage(Uno,enL,speed); % Set left motor to input speed 
    
    % Turn on Motor R Forward
    writeDigitalPin(Uno,in4,0); writeDigitalPin(Uno,in3,1); 
    writePWMVoltage(Uno,enR,speed-.1); % Set right motor to input speed (adjusted to go straight) 
    pause(distance) %tweek this time based on motor speed
     
    fprintf('Stop  -----  Stop\n\n');
    writePWMVoltage(Uno,enL,0); % Set speed on L to zero
    writePWMVoltage(Uno,enR,0); % Set speed on R to zero
    %------------- END OF CODE --------------
end

function ReverseTurnAndDrive(speed,desirAngle,distance,Uno,in1,in2,in3,in4,enL,enR,imu)
    % TurnAndDrive - Turn to an inputed angle, then go an inputed distance
    % Inputs: input1 - speed(voltage of motors),angle(in
    % degrees,distance(m), Uno(arduino
    % object),in1,in2,in3,in4,enL,enR(defined pins
    % Outputs: N/A
    %
    % Author: Preston Ernst
    % Collaborators: N/A
    % Date: 04/14/2021; Last revision: 14-April-2021
    %------------- BEGIN CODE --------------
    fprintf('Backward ----- Backward\n');
    % Turn on Motor L Backward
    writeDigitalPin(Uno,in1,1); writeDigitalPin(Uno,in2,0); 
    writePWMVoltage(Uno,enL,speed); % Set left motor to input speed 
    
    % Turn on Motor R Backward
    writeDigitalPin(Uno,in4,1); writeDigitalPin(Uno,in3,0); 
    writePWMVoltage(Uno,enR,speed); % Set right motor to input speed (adjusted to go straight) 
    pause(distance) %tweek this time based on motor speed
     
    fprintf('Stop  -----  Stop\n\n');
    writePWMVoltage(Uno,enL,0); % Set speed on L to zero
    writePWMVoltage(Uno,enR,0); % Set speed on R to zero
        pause(2)
    
    i = 1;
    for go = 1:200
    tic

    yaw(1) = 0;
    if abs(yaw(i)-desirAngle) < 5  % If you are close to the desired shut off motors and leave loop
        fprintf('Stop  -----  Stop\n\n');
        writePWMVoltage(Uno,enL,0); % Set speed on L to zero
        writePWMVoltage(Uno,enR,0); % Set speed on R to zero
        break
    elseif yaw(i) < desirAngle
        fprintf('Backward -----  Forward\n');
        % Turn on Motor L Backward
        writeDigitalPin(Uno,in1,1); writeDigitalPin(Uno,in2,0); 
        writePWMVoltage(Uno,enL,2.1);  % Set left motor to input speed 

        % Turn on Motor R Forward
        writeDigitalPin(Uno,in4,0); writeDigitalPin(Uno,in3,1); 
        writePWMVoltage(Uno,enR,2.2); % Set right motor to input speed (adjusted to go straight)
    elseif yaw(i) > desirAngle
        % Turn Right
        fprintf('Forward  ----- Backward\n');
        % Turn on Motor L Forward
        writeDigitalPin(Uno,in1,0); writeDigitalPin(Uno,in2,1); 
        writePWMVoltage(Uno,enL,2.2);  % Set left motor to input speed 

        % Turn on Motor R Backward
        writeDigitalPin(Uno,in4,1); writeDigitalPin(Uno,in3,0); 
        writePWMVoltage(Uno,enR,2.2);
    end

    gyroReading(i,:) = 180/pi * readAngularVelocity(imu);   % Turn AngularVelocity(rad/s)data into (degrees/s)
    gyroReading(i,3) = (gyroReading(i,3)  - 0.6501); % Take out error in data reading
    yaw(i+1) = yaw(i) + gyroReading(i,3)*toc;        % yaw (degrees) from start equals (degrees/s) times elapsed time
    disp(yaw(i))
    i = i + 1;
    end
    %------------- END OF CODE --------------
end
%------------- END OF CODE --------------