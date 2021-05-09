function out = mat2ocv(img)
% Convert image data format to OpenCV compatible image format
% Copyright 2020 The MathWorks, Inc.
out = permute(img(:,:,[3 2 1]),[3 2 1]);
end