close all
clear all

% read in 2 images
color1 = imread('DanaHallWay2/DSC_0286.JPG'); %hallway
color2 = imread('DanaHallWay2/DSC_0287.JPG');

%color1 = imread('DanaOffice/DSC_0313.JPG'); %bookshelf
%color2 = imread('DanaOffice/DSC_0314.JPG');

image1 = rgb2gray(color1);
image2 = rgb2gray(color2);

% apply Harris corner detection
corners1 = harris(image1,1,4,25000,0); 
corners2 = harris(image2,1,4,25000,0);

% show detected corners
figure(1)
imshow(color1)
hold on
plot(corners1(:,2),corners1(:,1),'r*')
hold off
figure(2)
imshow(color2)
hold on
plot(corners2(:,2),corners2(:,1),'r*')
hold off

% normalized cross correlation
correspondences = [];
for i = 1:size(corners1,1)
    
    y1 = corners1(i,1);
    x1 = corners1(i,2);
    %"radius" of patch, patch length = 2*patch_size
    patch_size = 10;
    
    %ignore corners with insufficient window area
    if x1 <= patch_size || y1 <= patch_size || x1 >= size(image1,2)-patch_size || y1 >= size(image1,1)-patch_size
        continue
    end
    
    %creates patch around each corner for both images
    patch1 = image1(y1-patch_size:y1+patch_size,x1-patch_size:x1+patch_size);
    NCC = zeros(1,size(corners2,1));
    for j = 1:size(corners2,1)
        y2 = corners2(j,1);
        x2 = corners2(j,2);

        if x2 <= patch_size || y2 <= patch_size || x2 >= size(image2,2)-patch_size || y2 >= size(image2,1)-patch_size
            continue
        end
        patch2 = image2(y2-patch_size:y2+patch_size,x2-patch_size:x2+patch_size);

        correlation = normxcorr2(patch1, patch2);
        
        %extract center value of NCC matrix for correlation value between
        %two patches
        NCC(j) = correlation(1+2*patch_size,1+2*patch_size);
        
        if NCC(j) == max(NCC)
            corr_index = j;
        end
    end
    
    %check threshold
    NCC_threshold = 0.90;
    if NCC(corr_index) > NCC_threshold
        %correspondences = [x1 y1 x2 y2]
        correspondences = [correspondences;x1 y1 corners2(corr_index,2) corners2(corr_index,1)];
    end
end

figure(3)
imshow([color1;color2])
hold all
plot(corners1(:,2), corners1(:,1), 'rx');
plot(corners2(:,2), corners2(:,1)+size(image1,1), 'bx');
for i = 1:size(correspondences,1)
    plot([correspondences(i,1), correspondences(i,3)], ...
        [correspondences(i,2), correspondences(i,4)+size(image1,1)]);
end
hold off

% homography function with RANSAC
mP1 = correspondences(:,1:2);
mP2 = correspondences(:,3:4);
[tform, inlierPoints1, inlierPoints2] = estimateGeometricTransform(mP1,mP2,'projective');

% determine homography inliers
figure(4)
imshow([color1;color2])
hold on
scatter(mP1(:,1), mP1(:,2), 'rx')
scatter(mP2(:,1), mP2(:,2)+size(image1,1), 'bx')
scatter(inlierPoints1(:,1), inlierPoints1(:,2),36, 'g', 'fill', 'LineWidth', 5);
scatter(inlierPoints2(:,1), inlierPoints2(:,2)+size(image1,1),36, 'g', 'fill','LineWidth', 5);
for i = 1:size(inlierPoints1,1)
    plot([inlierPoints1(i,1), inlierPoints2(i,1)], ...
        [inlierPoints1(i,2), inlierPoints2(i,2)+size(image1,1)],'g');
end
hold off

% mosaic
homo = imwarp(color1, tform);

% warped vs original
figure(5)
imshowpair(color2, homo, 'montage');
T = maketform('projective', tform.T);

% stitch
[stitched_image, stitched_mask, im1, im2] = stitch(color2,color1,T);
figure(6)
imshow(stitched_image)

%%
% fun with image warping
insert = imread('foreground.png');
background = imread('background3.png');
figure(7)
imshow(background)
hold on
[x,y]=ginput(4); %input clockwise starting from top left
%plot(x,y,'gx')
%hold off
pic_map = [1 1;size(insert,2) 1;size(insert,2) size(insert,1);1 size(insert,1)];
surface_map = [x y];%[1 1;100 100;100 200;1 300];
[tform, ~, ~] = estimateGeometricTransform(pic_map,surface_map,'projective');
warped_insert = imwarp(insert,tform);

% replace surface with image
% shift ginput bounds
polygon_bounds = [surface_map(:,1)-min(surface_map(:,1))+1 surface_map(:,2)-min(surface_map(:,2))+1];
warped_mask = zeros(ceil(fliplr(max(polygon_bounds))));
for i = 1:size(warped_mask,1)
    warped_mask(i,:) = inpolygon(repmat(i,1,size(warped_mask,2)),1:size(warped_mask,2),polygon_bounds(:,2),polygon_bounds(:,1));
end
% overlay image
coord = round(min(surface_map));%top left
for i = 1:size(warped_mask,1)
    for j = 1:size(warped_mask,2)
        if warped_mask(i,j)==1
            background(coord(2)+i-1,coord(1)+j-1,:) = warped_insert(i,j,:);
        end
    end
end
imshow(background)
hold off
