function houghLines(inputImage)
image= rgb2gray(inputImage);
image = image>100;

[H,theta,rho] = hough(image);

figure
imshow(imadjust(mat2gray(H)),[],...
       'XData',theta,...
       'YData',rho,...
       'InitialMagnification','fit');
title('Extracted lines');
xlabel('\theta (degrees)')
ylabel('\rho')
axis on
axis normal 
hold on
colormap(gca,hot)

end