function image = laplacian(inputImage)

mask = [0 -1 0;
        -1 4 -1;
        0 -1 0];
if (ndims(inputImage) ==3)
image = rgb2gray(inputImage);
image = conv2(image,mask, 'same');

else
    image = conv2(inputImage,mask);
end
    
end