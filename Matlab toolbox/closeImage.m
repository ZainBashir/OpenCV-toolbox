function image = closeImage(inputImage)

filter = closeDialog;

size = str2num (filter{1});
se = strel('square',size);

if (islogical(inputImage))
    image = imclose(inputImage,se);

elseif (ndims(inputImage) == 3)
    image = rgb2gray(inputImage);
    image = image<100;
    image = imclose(image,se);

else
    image = inputImage<50;
    image = imclose(image,se);

end
    
end