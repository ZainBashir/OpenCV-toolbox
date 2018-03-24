function H = computeHomography (I2, I1)



points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);

[f1, vpts1] = extractFeatures(I1, points1);

[f2, vpts2] = extractFeatures(I2, points2);

indexPairs = matchFeatures (f1,f2);

matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));

matchedPoints1 = matchedPoints1.Location;
matchedPoints2 = matchedPoints2.Location;

pts1 = matchedPoints1';
pts2 = matchedPoints2';

n = size(pts1,2);
A = zeros(2*n,9);
A(1:2:2*n,1:2) = pts1';
A(1:2:2*n,3) =1;
A(2:2:2*n,4:5) = pts1';
A(2:2:2*n,6) = 1;

x1 = pts1(1,:)';

y1 = pts1(2,:)';

x2 = pts2(1,:)';

y2 = pts2(2,:)';

A(1:2:2*n,7) = -x2.*x1;
A(1:2:2*n,7) = -y2.*x1;
A(1:2:2*n,8) = -x2.*y1;
A(1:2:2*n,8) = -y2.*y1;
A(1:2:2*n,9) = -x2;
A(1:2:2*n,9) = -y2;

[evec, ~] = eig (A'*A);
H = reshape (evec(:,1), [3,3])';
H = H/H ;

%% Stitching

stichedImage =I1;
stichedImage = padarray(stichedImage, [0 size(I2,2)], 0, 'post');
stichedImage = padarray(stichedImage, [size(I2,1) 0], 0, 'both');

mask1 = ones(size(I1));
mask1 = padarray(mask1, [0 size(I2,2)], 0, 'post');
mask1 = padarray(mask1, [size(I2,1) 0], 0, 'both');

mask2 = ones(size(mask1));

for i = 1:size(stichedImage,2)
    for j = 1:size(stichedImage,1)
        p2 = H * [ i; j-floor(size(I2,1)); 1];
        p2 = p2./p2(3);
        
        x2 = floor(p2(1));
        y2 = floor(p2(2));
        
        if x2> 0 && x2 <= size(I2,2) && y2 > 0 && y2 <= size(I2,1)
            stichedImage(j,i) = I2(y2,x2);
            mask2(j,i) = 0;
        end
    end
end

figure;
imshow(stichedImage);
