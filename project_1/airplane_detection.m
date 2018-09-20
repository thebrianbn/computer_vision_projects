% Main Function

% Pre-processing for base image
img = imread('dma.jpg');
gimg = grayimg(img);
gimg = filtershadow(gimg);
gimg = blur_img(gimg,2);



% Pre-processing for template 2
template2 = imread('template2.jpg');
template2 = grayimg(template2);
template2 = filtershadow(template2);
template2 = blur_img(template2,2);
% 

template33 = imresize(template2,.5);
template44 = imresize(template2,.25);


% 
% Object detection for template 2 on base image
new_m2 = get_corr_peaks(gimg, template2);
[x2,y2] = get_xy(new_m2, .5, template2);

new_m33 = get_corr_peaks(gimg, template33);
[x33,y33] = get_xy(new_m33, .5, template33);

new_m44 = get_corr_peaks(gimg, template44);
[x44,y44] = get_xy(new_m44, .4, template44);


% Append all detected objects to array
xs = [x2;x33;x44];
ys = [y2;y33;y44];

%dump template 1 and 3 and 5

% Visualize all detected objects on base image
display_points(xs,ys,gimg)


function [gimg] = grayimg(img)
%% Convert input image to grayscale %%
    img = double(img);
    gimg = (img(:,:,1)+img(:,:,2)+img(:,:,3))/3;
end

function [gimg] = filtershadow(gimg)
%% Remove shadows from image %%
b = size(gimg);
for row = 1:b(1)
    for col = 1:b(2)
        if gimg(row,col) < 30
            gimg(row,col) = 30;
        end
    end
end
end

function [img] = blur_img(img, sigma)
%% Apply Gaussian filter to image %%
img = imgaussfilt(img,sigma);
end

function [new_m] = get_corr_peaks(gimg, template_gimg)
%% Apply cross correlation and non-maximum suppression and find peaks %%
corr_vals = normxcorr2(template_gimg, gimg);

temp = imregionalmax(corr_vals);
[rows, columns] = size(temp);
new_m = zeros(rows,columns);
for row = 1: rows
    for col = 1: columns
        if temp(row,col) == 1
            new_m(row,col) = corr_vals(row,col);
        end
    end
end
end

function [x,y] =  get_xy(m, threshold, template_gimg)
%% Return x, y values of pixel values exceeding threshold %%
[x, y] = find(m>threshold);
for i = 1: length(x)
    x(i) = x(i) - (size(template_gimg, 1)/2);
    y(i) = y(i) - (size(template_gimg, 2)/2);
end
end

function display_points(x,y,gimg)
%% Visualize object-detected image %%
imagesc(gimg);
hold on
markers = 1:length(x);
fontsize = 6;
plot(y(1:length(x)), x(1:length(y)), 'r*','MarkerSize', 5)
text(y,x, cellstr(num2str(markers(:))), 'FontSize', fontsize, 'Color', 'red');

hold off
end