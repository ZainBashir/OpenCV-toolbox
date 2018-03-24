function image = cannyEdge(inputImage);

if (ndims(inputImage) == 3)
    display(ndims(inputImage));
     image = rgb2gray(inputImage);
     image = edge(image,'Canny');
    
else
    image = edge(inputImage,'Canny');
    
end
end