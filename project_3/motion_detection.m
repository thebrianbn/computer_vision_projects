main();

function main()
    % Main function to run the four motion detection algorithms on each
    % video directory.

    video_dirs = ["ArenaA", "ArenaN", "ArenaW", "getin", "getout", "movecam", "trees", "walk"];

    for i = 1:size(video_dirs, 2)
        motion_detection_frames(video_dirs(i));
    end
end


function motion_detection_frames(directory)
    % Run the four motion detection algorithms on the given video
    % directory.

    % Set directory strings
    full_dir = char(strcat('Project3-DataSets\DataSets\', directory));
    files = dir(strcat(full_dir, '\*.jpg'));
    
    % Get pixel dimensions of the video
    base_img = imread(strcat(full_dir, '\f0001.jpg'));
    pixelX = size(base_img, 1);
    pixelY = size(base_img, 2);
    
    % Set matrices
    imgs = zeros(size(files,1),pixelX,pixelY,3);
    gimgs = zeros(size(files,1),pixelX,pixelY,1);
    
    % Load in images and convert to grey-scale
    for i = 1: size(files,1)
        filename = strcat(full_dir, '\', files(i).name);
        imgs(i, :, :, :) = imread(filename);
        gimgs(i,:,:) =  (imgs(i,:,:,1)+imgs(i,:,:,2)+imgs(i,:,:,3))/3;
    end

    % Run the four motion detection algorithms on the frames
    M1 = simple_background_subtraction(gimgs, 60);
    M2 = simple_frame_differencing(gimgs, 60);
    M3 = adaptive_background_subtraction(gimgs, .5, 60);
    M4 = persistent_frame_differencing(gimgs, 60, 50);
    
    % Final matrix for four-panel
    Mfinal = zeros(size(M1, 1), pixelX*2, pixelY*2);
    
    % Create four-panel of output videos
    for z  = 1:size(M1, 1)
        Mfinal(z,:,:) = [squeeze(M1(z,:,:)) squeeze(M2(z,:,:)); squeeze(M3(z,:,:)) squeeze(M4(z,:,:))];
    end

    % Output the four-panel frame files as jpgs
    for i = 1:size(Mfinal,1)
        filename = strcat(char(directory), int2str(i), '.jpg');
        imwrite(squeeze(Mfinal(i,:,:)), filename);
    end
end

function M = adaptive_background_subtraction(gimgs, alpha, threshold)
    % Run adaptive background subtraction on the given grey-scaled frames.
    
    B = zeros(size(gimgs));
    M = zeros(size(gimgs));
    framediff = zeros(size(gimgs));
    B(1,:,:) = gimgs(1,:,:);
    
    for t = 2:size(gimgs,1)
        framediff(t,:,:) = abs(B(t-1,:,:)-gimgs(t,:,:));
        M(t,:,:) = (framediff(t,:,:) > threshold)*255;
        B(t,:,:) = alpha* gimgs(t,:,:) + (1-alpha)* B(t-1, :, :);    
    end
end


function [frames_result] = simple_background_subtraction(frames, threshold)
    % Run simple background subtraction on the given grey-scaled frames.

    static_image = frames(1, :, :, :);
    frames_result = zeros(size(frames));
    
    for x = 1:size(frames,1)
        diff = abs(static_image - frames(x, :, :, :));
        frames_result(x, :, :)= diff > threshold;
    end
        
end


function [frames_result] = simple_frame_differencing(frames, threshold)
    % Run simple frame differencing on the given grey-scaled frames.

    frames_result = zeros(size(frames));
    
    for x=2:size(frames, 1)
        diff = abs(frames(x-1, :, :) - frames(x, :, :));
        frames_result(x, :, :) = diff >threshold;
    end
end

function [H] = persistent_frame_differencing(frames, threshold, gamma)
    % Run persistent frame differencing on the given grey-scaled frames.

    B = zeros(size(frames));
    H = zeros(size(frames));
    M = zeros(size(frames));
    framediff = zeros(size(frames));
    B(1,:,:) = frames(1,:,:);
    
    for t=2:size(frames,1)
        framediff(t,:,:) = abs(B(t-1,:,:)-frames(t,:,:));
        M(t,:,:) = (framediff(t,:,:) > threshold);
        temp = max(H(t-1,:,:) - gamma,0);
        H(t,:,:) = max(255 * M(t,:,:) , temp);
        B(t,:,:) = frames(t,:,:);
    end
end
