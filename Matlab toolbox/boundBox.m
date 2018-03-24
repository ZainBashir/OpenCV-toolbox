function boundBox (inputImage)

filter  = boundDialog;
threshold = str2num(filter{1});
numBoxes = str2num(filter{2});

inputImage = rgb2gray(inputImage) >threshold;

box=regionprops(inputImage,'Area', 'BoundingBox'); 

for i = 1:length(box)
    areas (i) = box(i).Area;
end

[~, indices] = sort(areas, 'descend');

if (numBoxes > length(areas))
    errordlg('The number of boxes specified exceeds the number of boxes detected', 'Error');
end

for i = 1: numBoxes;
    
    thisBB = box(indices(i)).BoundingBox;
    rectangle('Position', [thisBB(1),thisBB(2),thisBB(3),thisBB(4)],...
  'EdgeColor','g','LineWidth',2 );
end
hold on