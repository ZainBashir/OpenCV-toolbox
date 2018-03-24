function cornersFAST(inputImage)


    prompt = {'Enter the number of strongest circles you want to find'};
     num_lines =1;
     dlg_title = 'Circles';
     defaultans = {'1'};
     filter = inputdlg(prompt,dlg_title,num_lines,defaultans); 
    points = str2num(filter{1});
    image= rgb2gray(inputImage);
    corners =detectFASTFeatures(image);

if(points> corners.Count)
    errordlg('Try specifying a lesser number of points');
     h = findobj('Tag','undoButton');
     undoButton_Callback(h.hObject,eventdata,handles);

else

hold on;
plot(corners.selectStrongest(points));
end

end
