function params = calibCamera (imageFileNames,images)

[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

squareSizeInMM = 30;
worldPoints = generateCheckerboardPoints(boardSize,squareSizeInMM);

I = readimage(images,1);
imageSize = [size(I, 1),size(I, 2)];
params = estimateCameraParameters(imagePoints,worldPoints);



hold on;
plot(imagePoints(:,1,1), imagePoints(:,2,1),'go');
plot(params.ReprojectedPoints(:,1,1),params.ReprojectedPoints(:,2,1),'r+');
legend('Detected Points','ReprojectedPoints');

end