clear all;

% checkerboard generator

resX = 1920
resY = 1200

aspect = resY / resX

numCheckerColumn = 50
numCheckerRow = floor(numCheckerColumn * aspect)

intervalX = resX / numCheckerColumn
intervalY = resY / numCheckerRow

I = zeros(resY,resX);

for c=1:2:numCheckerColumn
    for r=1:2:numCheckerRow
        I((r-1) * intervalY + 1:r * intervalY,(c-1) * intervalX + 1:c * intervalX) = 255;
    end
end

for c=2:2:numCheckerColumn
    for r=2:2:numCheckerRow
        I((r-1) * intervalY + 1:r * intervalY,(c-1) * intervalX + 1:c * intervalX) = 255;
    end
end

I_uint8(:,:,1) = uint8(I);
I_uint8(:,:,2) = uint8(I);
I_uint8(:,:,3) = uint8(I);

% draw central marker
center = floor([resX / 2, resY / 2])
centerSize = 2

for i = center(1)-centerSize:1:center(1)+centerSize
    for j= center(2)-centerSize:1:center(2)+centerSize
        x = floor(i)
        y = floor(j)
        I_uint8(y,x,1) = 255;
        I_uint8(y,x,2) = 0;
        I_uint8(y,x,3) = 0;
    end
end

% mark corner points


imshow(I_uint8);

title = sprintf('checkerboard_%dx%d_%dx%d.png', numCheckerColumn, numCheckerRow, resX, resY)
imwrite(I_uint8,title);


