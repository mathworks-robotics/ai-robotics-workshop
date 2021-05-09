function jetBotKeyboardControl
%exampleHelperTurtleBotKeyboardControl - Allows for user control of JetBot through keyboard commands
%   exampleHelperJetBotKeyboardControl(HANDLES) takes a set of subscriber
%   and publisher handles as input and publishes velocity commands according
%   to user keyboard input. The JetBot location is plotted.
%
%   See also JetBotTeleoperationExample

%   Copyright 2020 The MathWorks, Inc.

camSub = rossubscriber('/image_raw','DataFormat','struct');
velPub = rospublisher("/jetbot_motors/cmd_raw","std_msgs/String");

hFig = uifigure('Name','JetBot ROS Teleop',...
    'KeyPressFcn',@keyPressFcn,'KeyReleaseFcn',@keyReleaseFcn);
uilabel(hFig,'Position',[50,80,50,50]*2 + [hFig.Position(3:4)/4 0 0],'Text','i = Forward');
uilabel(hFig,'Position',[50,40,50,50]*2 + [hFig.Position(3:4)/4 0 0],'Text','k = Backward');
uilabel(hFig,'Position',[25,60,50,50]*2 + [hFig.Position(3:4)/4 0 0],'Text','j = Left');
uilabel(hFig,'Position',[100,60,50,50]*2 + [hFig.Position(3:4)/4 0 0],'Text','l = Right');
uilabel(hFig,'Position',[50,20,50,50]*2 + [hFig.Position(3:4)/4 0 0],'Text','q = Quit');
uilabel(hFig,'Position',[50,60,50,50]*2 + [hFig.Position(3:4)/4 0 0],'Text','s = Save image');
uilabel(hFig,'Position',[50,0,500,50]*2 + [hFig.Position(3:4)/4 0 0],'Text','Keep this figure in scope to give commands');

pressedKeys = {};
d = 0.05125*2;
r = 0.03;

disp('Keyboard Control: ');
disp('i=forward, k=backward, j=left, l=right');
disp('q=quit');
disp('Waiting for input: ');

% Video player
videoPlayer = vision.DeployableVideoPlayer;

% Get current date and time
dateprefix = datestr(now,'YYYY_mm_DD_HHMMss');

% Create a video file
videFileNmae = fullfile(['trainVideo_',dateprefix,'.avi']);
writer = vision.VideoFileWriter(videFileNmae,...
    'FileFormat','AVI','FrameRate',5);

rate = rateControl(10);
while ~any(strcmp(pressedKeys,'q')) && ishandle(hFig)
    turnV = 0;
    forwardV = 0;
    
    for k = 1:numel(pressedKeys)
        reply = pressedKeys{k};
        switch reply
            case 'i'         % i
                forwardV = 0.3;
            case 'k'     % k
                forwardV = -0.3;
            case 'j'     % j
                turnV = pi*1.5;
            case 'l'     % l
                turnV = -pi*1.5;
            case 's'
                writer(I);
            otherwise
                turnV = 0;
                forwardV = 0;
        end
    end
    
    if numel(pressedKeys) == 0
        turnV = 0;
        forwardV = 0;
    end

    % Publish velocities to the robot
    exampleHelperJetBotSetVelocity(velPub, forwardV, turnV);
    
    % Subscribe image
    imMsg = receive(camSub);
    I = rosReadImage(imMsg);
    %pos = predict(handles.trainedNet,I);
    %pos = (pos + 1) .* (320/2);
    %Iout = insertShape(I, 'FilledCircle', [pos 5],'Color','Green');
    Iout = I;
    videoPlayer(Iout);
    
    waitfor(rate);
end
release(writer);
if ishandle(hFig)
    close(hFig);
end

    function keyPressFcn(~, event)
        pressedKeys = unique([pressedKeys, {event.Key}]);
    end

    function keyReleaseFcn(~, event)
        pressedKeys(strcmp(pressedKeys,event.Key)) = [];
    end


    function exampleHelperJetBotSetVelocity(velPub, vLin, vAng)
        %exampleHelperJetBotSetVelocity Sets linear and angular velocity of JetBot
                
        persistent strMsg
        
        if isempty(strMsg)
            strMsg = rosmessage(velPub);
        end
        
        vl = vLin - d*vAng/2;
        vr = vLin + d*vAng/2;
        [wlCmd, wrCmd] = vel2cmd(vl,vr);
        
        strMsg.Data = "speed[ " + wlCmd + "," + wrCmd +" ]";
        send(velPub,strMsg);
        
        function [wlCmd, wrCmd] = vel2cmd(vl,vr)
            wl = vl / r;
            wr = vr / r;
            wlCmd = funcl(wl);
            wrCmd = funcr(wr);
        end
        
        function wCmd = funcl(w)
            wCmd = min(max(w*0.1,-1),1);
        end
        function wCmd = funcr(w)
            %wCmd = min(max(w*0.096,-1),1);
            wCmd = min(max(w*0.1,-1),1);
        end
    end

end

