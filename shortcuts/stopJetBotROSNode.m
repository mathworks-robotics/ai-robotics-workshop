function stopJetBotROSNode
if exist('jetsonIpAddr.mat','file')
    d = load('jetsonIpAddr.mat');
    jetsonIpAddr = d.jetsonIpAddr;
else
    errordlg('launchJetBotROSNodeが一度も実行されていません。');
end

d = rosdevice(jetsonIpAddr,'jetbot','jetbot');
cmd = 'pkill roslaunch';
try
    ret = system(d,cmd);
catch
    errordlg('ROSノードのプロセスが見つかりません');
end
end