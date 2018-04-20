I = imread('pic1.jpg');
C = imread('checkerboard_20x14_1024x720.png');



% click points
cpselect(I,C); % moving image, fixed image

%% rescale input

screenSizeFixed = size(C);
screenSizeMoving = size(I);

movingImageResized = imresize(I,[screenSizeFixed(1) screenSizeFixed(2)]);


% rescale moving points dimension to fixed image space
movingPointsScaled = movingPoints;
movingPointsScaled(:,1) = movingPointsScaled(:,1) / screenSizeMoving(2) * screenSizeFixed(2);
movingPointsScaled(:,2) = movingPointsScaled(:,2) / screenSizeMoving(1) * screenSizeFixed(1);

fixedPointsScaled = fixedPoints;


%%

%plot(x,y,'g*-');

%h = cpselect(I,C)

% create a mesh grid

%interpolationMethod = 'nearest';
%interpolationMethod = 'linear';
interpolationMethod = 'cubic';
%interpolationMethod = 'v4';

interval = 1

[xq,yq] = meshgrid(1:interval:screenSizeFixed(2),1:interval:screenSizeFixed(1));

x = movingPointsScaled(:,1);
y = movingPointsScaled(:,2);

% deformation in x
vx = fixedPointsScaled(:,1) - movingPointsScaled(:,1); % displacement in x
%vx = movingPointsScaled(:,1) - fixedPointsScaled(:,1); % displacement in x

vx = vx / screenSizeFixed(2);
vqX = griddata(x,y,vx,xq,yq,interpolationMethod);
D(:,:,1) = vqX;

% deformation in y
vy = fixedPointsScaled(:,2) - movingPointsScaled(:,2); % displacement in x
%vy = movingPointsScaled(:,2) - fixedPointsScaled(:,2); % displacement in x

vy = vy / screenSizeFixed(1);
vqY = griddata(x,y,vy,xq,yq,interpolationMethod);
D(:,:,2) = vqY;

% write output
csvwrite('deformationX.txt',D(:,:,1));
csvwrite('deformationY.txt',D(:,:,2));

%

% Plot the gridded data as a mesh and the scattered data as dots. 
figure;
mesh(xq,yq,D(:,:,1))
hold on
plot3(x,y,vx,'o')
%xlim([0 screenSize(2)])
%ylim([0 screenSize(1)])
title('deformation in x');

% Plot the gridded data as a mesh and the scattered data as dots. 
figure;
mesh(xq,yq,D(:,:,2))
hold on
plot3(x,y,vy,'o')
%xlim([0 screenSize(2)])
%ylim([0 screenSize(1)])
title('deformation in y');

%

%% apply image deformation



deformedImage = uint8(zeros(size(C)));

for idxX = 1:screenSizeFixed(2)
    for idxY = 1:screenSizeFixed(1)
        
        idxXNormalized = idxX / screenSizeFixed(2);
        idxYNormalized = idxY / screenSizeFixed(1);
       
        multiplier = 1.0;
        
        shiftedCoordX = (idxXNormalized + D(idxY,idxX,1) * multiplier) * screenSizeFixed(2);
        shiftedCoordY = (idxYNormalized + D(idxY,idxX,2) * multiplier) * screenSizeFixed(1);

        % nearest neighbor lookup
        shiftedCoordX = floor(shiftedCoordX);
        shiftedCoordY = floor(shiftedCoordY);
                
        if (shiftedCoordX > 0 && shiftedCoordX <= screenSizeFixed(2)) % check bounds X
            if (shiftedCoordY > 0 && shiftedCoordY <= screenSizeFixed(1))  % check bounds Y
                % write into shifted coordinates
                deformedImage(shiftedCoordY, shiftedCoordX, :) = movingImageResized(idxY, idxX, :);
                %deformedImage(idxY, idxX, :) = movingImageResized(shiftedCoordY, shiftedCoordX, :);
            end
        end
      
%         shiftedCoordX = floor(shiftedCoordX);
%         shiftedCoordY = floor(shiftedCoordY);
%                 
%         if (shiftedCoordX > 0 && shiftedCoordX <= screenSizeFixed(2)) % check bounds X
%             if (shiftedCoordY > 0 && shiftedCoordY <= screenSizeFixed(1))  % check bounds Y
%                 % write into shifted coordinates
%                 deformedImage(shiftedCoordY, shiftedCoordX, :) = movingImageResized(idxY, idxX, :);
%                 %deformedImage(idxY, idxX, :) = movingImageResized(shiftedCoordY, shiftedCoordX, :);
%             end
%         end

        
    end
end

%figure,imshow(deformedImage);

figure,imshow(deformedImage .* 0.5 + C .* 0.2);


