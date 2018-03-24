function image = dialate(inputImage)

filter = dilateDialog;

size = str2num (filter{1});
se = strel('square',size);

if (islogical(inputImage))
    image = imdilate(inputImage,se);

elseif (ndims(inputImage) == 3)
    image = rgb2gray(inputImage);
    image = image<100;
    image = imdilate(image,se);

else
    image = inputImage<50;
    image = imdilate(image,se);

end
    
end