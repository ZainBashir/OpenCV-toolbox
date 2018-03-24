function image = addLogo(logo, image)

% %// Load in MATLAB logo without alpha channel
% logo = imread('Flag_of_Pakistan.svg.png');
% 
% %// Load in Jack Bauer
% jack = imread('lena.jpg');

%// Resize the MATLAB logo
logoResize = imresize(logo, 0.35, 'bilinear');

%// Get the size of the resized logo - we need this
%// to properly mix the stuff in
rows = size(logoResize, 1);
cols = size(logoResize, 2);

%// Specify alpha here
alpha = 0.8;

%// Mix in the logo with the image
image(end-rows+1:end,end-cols+1:end,:) = uint8(alpha.*double(logoResize) + ...
(1-alpha).*double(image(end-rows+1:end,end-cols+1:end,:)));

% figure; 
% imshow(image);