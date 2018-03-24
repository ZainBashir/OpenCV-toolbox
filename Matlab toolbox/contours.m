function contours(inputImage)

 A = rgb2gray(inputImage)>100;
 
 boundaries =  bwboundaries(A);

 hold on
 for k = 1:length(boundaries)
   boundary = boundaries{k};
   plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 2)
 end
