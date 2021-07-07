function [predictedPosNorm,Iout] = line_detection(I)
%#codegen
%   Copyright 2020 The MathWorks, Inc.

persistent lineDetObj;
NetworkInputSize = [180 320 3];

if isempty(lineDetObj)
    lineDetObj = coder.loadDeepLearningNetwork('mynet_new.mat');
end

% Convert image data format OpenCV BGR to MATLAB compatible RGB image format
img = ocv2mat(I);

%img = imrotate(img,180);

sz = size(img);
sizeWH = sz([2 1]);

% Resize
in = imresize(img,NetworkInputSize(1:2));

% pass in input
predictedPosNorm = predict(lineDetObj,in);
predictedPos = (predictedPosNorm + 1) / 2 .* sizeWH(1);

% Visualize
out = insertShape(img,"Line",[sizeWH(1)/2 sizeWH(2) predictedPos],'Color',[255 0 0],'LineWidth',5);
out = insertShape(out,"FilledCircle",[predictedPos 5],'Color',[255 0 0],'Opacity',1);

% Convert image data format to OpenCV compatible image format
Iout = mat2ocv(out);

end
