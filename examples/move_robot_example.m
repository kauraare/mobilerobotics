% Example file demonstrating how to send movement commands to the Husky
%   Precondition: run all processes via Mission Control and press Y-A on
%   the remote control

% Add MRG helper functions
addpath('mrg');

% Set the Husky ID
husky_id = 4;

% Get the Husky configuration, including the MOOS channel names.
husky_config = GetHuskyConfig(husky_id);

% Initialise mex-moos and register subscribers
clear mexmoos;
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', husky_config.host, 'MOOSNAME', client, 'SERVERPORT','9000');
pause(1.0); % give mexmoos a chance to connect (important!)

% First tell it not to move at all
SendSpeedCommand(0, 0, husky_config.control_channel)


i = 0;
velocity = 0.3;

while true
    disp('Send command');
    mailbox = mexmoos('FETCH');
    
    angle = 0.3;
    
    if mod(i, 500) == 0
        velocity = -velocity;
    end
    
    i = i + 1;
    SendSpeedCommand(velocity, angle, husky_config.control_channel);
    pause(0.01); % don't overload moos w/commands
end

