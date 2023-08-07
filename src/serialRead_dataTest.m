%% Preparing the serial communication
% https://www.mathworks.com/help/matlab/import_export/read-streaming-data-from-arduino.html
% Clear the workspace
% Select the correct port and correct Baudrate
% Set the terminator to "CR/LF" (CR = Carriage Return and LF = Line Feed)
clc; clear; close all; format compact; format shortG
% s = serialport('COM12', 9600);
% configureTerminator(s, "CR/LF");

% Use this code for real-time data vizualisation (plot angle vs time)
% clc; close all; flush(s);

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

% setup time vector
t = linspace(0, 1e4*pi, 100000);
%setup fake data params
loadcell = 100*sin(t);
motor = 60*sin(t+2);
setPoint = 40*cos(t+1);

% add points and render 
for k = 1:length(t)
    %add points to the corresponding lines
    addpoints(h1, t(k), loadcell(k))
    addpoints(h2, t(k), setPoint(k))
    addpoints(h3, t(k), motor(k))
    
    % set xlim to move plot horizontally to the right
    % can also set ylim to scale plot upwards and down
    xlim([t(k)-10, t(k)+10])

    % draw and sleep
    drawnow
    java.lang.Thread.sleep(50);
end
%assign the variables
%set xlim to move our graph horizontally
% xlim([max(0, time/1000 - 10), time/1000 + 10]);
%set ylim to resize our graph upward

% writematrix([loadcell time/1000],'loadcell.xls','WriteMode','append')

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