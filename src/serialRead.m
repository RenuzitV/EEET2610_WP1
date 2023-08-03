%% Preparing the serial communication
% https://www.mathworks.com/help/matlab/import_export/read-streaming-data-from-arduino.html
% Clear the workspace
% Select the correct port and correct Baudrate
% Set the terminator to "CR/LF" (CR = Carriage Return and LF = Line Feed)
clc; clear; close all; format compact; format shortG
s = serialport('COM12', 9600);
configureTerminator(s, "CR/LF");

% Use this code for real-time data vizualisation (plot angle vs time)
clc; close all; flush(s);

% Create a variable to store user input
global userInput;
userInput = "";
% Prepare the parameters for the animated line
hFig = figure('KeyPressFcn', @keyPressCallback, 'Name', 'MATLAB Command Window');

h1 = animatedline('Color','b','LineWidth',2, 'MaximumNumPoints',500); 
h2 = animatedline('Color','r','LineWidth',2, 'MaximumNumPoints',500); 
h3 = animatedline('Color','g','LineWidth',2, 'MaximumNumPoints',500); 
grid on;
screen_property = get(0,'screensize');
set(gcf, "OuterPosition", [0, screen_property(4)/2, ...
    screen_property(3)/2, screen_property(4)/2])
xlabel("Time (s)");
ylim([-50 100]);

% Start the serial COM reading and animation
% Break the loop with Ctrl+C
while 1
    if (~strcmp(userInput, ''))
        writeline(s, userInput);
        userInput = '';
    end
    string = readline(s)
    data = sscanf(string, "Load_cell output val: %f\nsetpoint: %f\nmotor output: %f%%\ntime: %f\n");
    if (size(data) ~= 4) 
        continue;
    end
    loadcell = data(1);
    setPoint = data(2);
    motor = data(3);
    time = data(4);
    xlim([max(0, time/1000 - 10), time/1000 + 10]);
    ylim([-50, min(100, motor + 75)])
    addpoints(h1, time/1000, loadcell)
    addpoints(h2, time/1000, setPoint)
    addpoints(h3, time/1000, motor)
    % writematrix([loadcell time/1000],'loadcell.xls','WriteMode','append')
    drawnow
end

% Command Window KeyPressFcn callback function
function keyPressCallback(src, event)
    % Use the global variable 'userInput' inside the callback function
    global userInput;
    % Get the pressed key
    key = event.Key;

    % Handle the user input
    if (strcmp(key, 's') || strcmp(key, 't'))
        userInput = key;
    end
end