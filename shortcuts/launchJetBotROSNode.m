function launchJetBotROSNode

if exist('jetsonIpAddr.mat','file')
    d = load('jetsonIpAddr.mat');
    jetsonIpAddr = d.jetsonIpAddr;
else
    jetsonIpAddr = '192.168.1.245';
end
prompt = {'JetBot IP address:'};
dlgtitle = 'Input';
dims = [1 35];
definput = {jetsonIpAddr};
answer = inputdlg(prompt,dlgtitle,dims,definput);

if isempty(answer)
    return;
end

jetsonIpAddr = answer{1};

d = rosdevice(jetsonIpAddr,'jetbot','jetbot');


cmd = 'pkill roslaunch';
try
    ret = system(d,cmd);
    pause(5);
catch
    %warning('プロセスが見つかりません');
end

rosMasterURI = ['http://',jetsonIpAddr,':11311'];
rosIP = jetsonIpAddr;
catkinWs = '~/workspace/catkin_ws'; % launchファイルが含まれるワークスペース
nodeName = 'jetbot_ros';
logFile = [catkinWs '/' nodeName '.log'];

% 実行したいroslaunchファイル
cmdArgs = 'roslaunch jetbot_ros jetbot_ros.launch';

% コマンド列(他に必要な環境変数などあれば追加する)
cmd = ['export DISPLAY=:0.0; ' ...
    'export XAUTHORITY=~/.Xauthority; ' ...
    'export ROS_MASTER_URI=' rosMasterURI '; ' ...      % Export ROS master URI
    'export ROS_IP=' rosIP '; ' ...                  % Export ROS NodeHost
    'export SVGA_VGPU10=0; ' ... % VMware時のみ必要
    'source ' catkinWs '/devel/setup.bash; ' ...        % Source the Catkin workspace
    ' ' cmdArgs ' ' ...                                 % Run node executable
    '&> ' logFile ' &'...                               % Pipe all output into a log file
    ];

% システムコマンドの実行
system(d,cmd);
rosshutdown;
setenv('ROS_MASTER_URI',['http://',jetsonIpAddr,':11311']);
pause(5);

rosinit;
pause(2);

nodes = rosnode('list');
flag = any(strcmp(nodes,'/jetbot_camera')) & any(strcmp(nodes,'/jetbot_motors'));
if flag
    msgbox(['ROS node is running on ',jetsonIpAddr,' successfully!']);
else
    error(['ROS node is not starting on ',jetsonIpAddr]);
end

save('jetsonIpAddr','jetsonIpAddr');
end