function houghCircles(inputImage)

filter = circlesDialog;
circles = str2num(filter{1});

[centers, radii, metric] = imfindcircles(inputImage, [1 15000]);

if (circles > size(centers,1))
    errordlg('The number of circles specified is too large for this image');
    
    h = findobj('Tag','undoButton');
    undoButton_Callback(h.hObject,eventdata,handles);

else

 centersStrong5 = centers(1:circles,:);
radiiStrong5 = radii(1:circles);
metricStrong5 = metric(1:circles);
imshow(inputImage);
hold on;
viscircles(centersStrong5, radiiStrong5,'EdgeColor','b');
end 
end
