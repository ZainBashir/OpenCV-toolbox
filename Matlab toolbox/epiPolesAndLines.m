function [points1, points2] = epiPolesAndLines (image1, image2)

image1 = rgb2gray(image1);
image2 = rgb2gray(image2);


points1 = detectHarrisFeatures(image1);
points2 = detectHarrisFeatures(image1);

[features1,valid_points1] = extractFeatures(image1,points1);
[features2,valid_points2] = extractFeatures(image2,points2);

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

[fLMedS,inliers] = estimateFundamentalMatrix(matchedPoints1.Location,...
    matchedPoints2.Location,'NumTrials',4000);


epiLines = epipolarLine(fLMedS',matchedPoints2.Location(inliers,:));
points1 = lineToBorderPoints(epiLines,size(image1));



% line(points(:,[1,3])',points(:,[2,4])');




epiLines = epipolarLine(fLMedS,matchedPoints1.Location(inliers,:));
points2 = lineToBorderPoints(epiLines,size(image2));
% line(points(:,[1,3])',points(:,[2,4])');
% truesize;
