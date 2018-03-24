function image = erode(inputImage)

filter = erodeDialog;

size = str2num (filter{1});
se = strel('square',size);

if (islogical(inputImage))
    image = imerode(inputImage,se);

elseif (ndims(inputImage) == 3)
    image = rgb2gray(inputImage);
    image = image<100;
    image = imerode(image,se);

else
    image = inputImage<50;
    image = imerode(image,se);

end
    
end