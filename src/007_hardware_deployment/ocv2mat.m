function out = ocv2mat(img)
% Convert image data format OpenCV BGR to MATLAB compatible RGB image format
% Copyright 2020 The MathWorks, Inc.
out = permute(img,[3 2 1]);
out = out(:,:,[3 2 1]);
end