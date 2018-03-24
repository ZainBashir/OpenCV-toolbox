function enclosingCircle(inputImage)

filter  = circleDialog;
threshold = str2num(filter{1});
numCircles = str2num(filter{2});

inputImage = rgb2gray(inputImage) >threshold;

box=regionprops(inputImage,'Area', 'EquivDiameter', 'Centroid'); 
centers = [];
topCenters= [];
topRadii = [];
for i = 1:length(box)
    centers = [centers ;box(i).Centroid];
    radii (i) =  box(i).EquivDiameter;
end
radii = radii/2;
[~, indices] = sort(radii, 'descend');

if (numCircles > length(radii))
    errordlg('The number of circles specified exceeds the number of circles detected', 'Error');
end

for i = 1:numCircles
    topCenters (i,:) = centers (indices(i),:);
    topRadii(i) = radii (indices(i));
end

viscircles(topCenters,topRadii);
hold on;