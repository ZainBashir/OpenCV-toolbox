function image = openImage(inputImage)

filter = openDialog;

size = str2num (filter{1});
se = strel('square',size);

if (islogical(inputImage))
    image = imopen(inputImage,se);

elseif (ndims(inputImage) == 3)
    image = rgb2gray(inputImage);
    image = image<100;
    image = imopen(image,se);

else
    image = inputImage<50;
    image = imopen(image,se);

end
    
end