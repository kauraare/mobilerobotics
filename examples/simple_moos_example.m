% Very simple example to verify that your mexmoos is installed correctly

%% Clear any previous connections to MOOS
clear mexmoos;

%% Connect to MOOS
unique_num = num2str(int32(rand*1e7));
host = '192.168.0.14';
client = ['ExampleCdtClient' unique_num];
mexmoos('init', 'SERVERHOST', host, 'MOOSNAME', client, 'SERVERPORT','9000');
pause(1.0); % give mexmoos a chance to connect (important!)

%% Register (i.e., "listen") for messages on a specific MOOS channel
channel_name = ['CdtExampleChannel' unique_num];
mexmoos('REGISTER', channel_name, 0.0);

%% Send a message over the channel
disp(['Sending message over MOOS with unique number: ' unique_num]);
message_to_send = ['Hello, MOOS world (' unique_num ')'];
mexmoos('NOTIFY', channel_name, message_to_send);

%% See if we can read the same message back from MOOS
disp('Reading message from MOOS');
pause(0.1);
moos_mailbox = mexmoos('FETCH');
msg_idx = find(strcmp({moos_mailbox.KEY}, channel_name));
for idx = msg_idx
    disp(['Received Message:  "' moos_mailbox(idx).STR '"']);
end
if isempty(msg_idx)
    error('Did not receive message from MOOS!');
end

%% Cleanup
mexmoos('CLOSE');