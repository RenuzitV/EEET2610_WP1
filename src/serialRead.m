%% Preparing the serial communication
% https://www.mathworks.com/help/matlab/import_export/read-streaming-data-from-arduino.html
% Clear the workspace
% Select the correct port and correct Baudrate
% Set the terminator to "CR/LF" (CR = Carriage Return and LF = Line Feed)
clc; clear; close all; format compact; format shortG
s = serialport('COM14', 9600);
configureTerminator(s, "CR/LF");


% Use this code for real-time data vizualisation (plot angle vs time)
clc; close all; flush(s);

% Create a variable to store user input
global userInput;
userInput = "";
% Prepare the parameters for the animated line

%setup figure for our graph, most important thing is keyPressCallback,
%which listens for keys and sends commands to serial output
hFig = figure('KeyPressFcn', @keyPressCallback, 'Name', 'MATLAB Command Window');

%create 3 animated lines for our code
h1 = animatedline('Color','b','LineWidth',2, 'MaximumNumPoints',500); 
h2 = animatedline('Color','r','LineWidth',2, 'MaximumNumPoints',500); 
h3 = animatedline('Color','g','LineWidth',2, 'MaximumNumPoints',500); 
grid on;

%setup window size
screen_property = get(0,'screensize');
set(gcf, "OuterPosition", [0, screen_property(4)/2, ...
    screen_property(3)/2, screen_property(4)/2])
xlabel("Time (s)");

% Start the serial COM reading and animation
% Break the loop with Ctrl+C
while 1
    % write to serial
    if (~strcmp(userInput, ''))
        writeline(s, userInput);
        userInput = '';
    end
    %read from serial communication
    %do NOT add a semicolon so we can see its outputs via the command window
    string = readline(s);
    %scans the string for inputs: %f 
    %double %% means one % in the string
    if (size(string) == 0)
        continue;
    end
    disp(string)
    data = sscanf(string, "Load_cell output val: %f\nsetpoint: %f\nmotor output: %f%%\ntime: %f\n");
    %make sure the data is correct i.e. 4 outputs is extracted from string
    if (size(data) < 4) 
        continue;
    end
    %assign the variables
    loadcell = data(1);
    setPoint = data(2);
    motor = data(3);
    time = data(4);
    %set xlim to move our graph horizontally
    xlim([max(0, time/1000 - 10), time/1000 + 10]);
    %set ylim to resize our graph upward
    ylim([-5, min(250, round(setPoint/40)*40 + 40)])
    
    %add points to the corresponding lines
    addpoints(h1, time/1000, loadcell)
    addpoints(h2, time/1000, setPoint)
    addpoints(h3, time/1000, motor)
    % writematrix([loadcell setPoint time/1000],'loadcell3.xlsx','WriteMode','append')
    drawnow
end

% Command Window KeyPressFcn callback functions
function keyPressCallback(src, event)
    % Use the global variable 'userInput' inside the callback function
    global userInput;
    % Get the pressed key
    key = event.Key;

    % Handle the user input
    if (strcmp(key, 's') || strcmp(key, 't') || strcmp(key, '\n'))
        userInput = key;
    end
end