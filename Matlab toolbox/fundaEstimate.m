function F = fundaEstimate(image1, image2)

image1 = rgb2gray(image1);
image2 = rgb2gray(image2);


points1 = detectHarrisFeatures(image1);
points2 = detectHarrisFeatures(image1);

[features1,valid_points1] = extractFeatures(image1,points1);
[features2,valid_points2] = extractFeatures(image2,points2);

indexPairs = matchFeatures(features1,features2);

matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

F = estimateFundamentalMatrix(matchedPoints1,...
    matchedPoints2,'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);

end

