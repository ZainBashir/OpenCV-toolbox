function image = sobel(inputImage);

if (ndims(inputImage) == 3)
    display(ndims(inputImage));
     image = rgb2gray(inputImage);
     image = edge(image,'sobel');
    
else
    image = edge(inputImage,'sobel');
    
end
end