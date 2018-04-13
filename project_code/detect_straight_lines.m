function [YPDist, YPAng , WLDist, WLAng] = detect_straight_lines(img)

% clearvars -except img

%% Calculate distance between camera and object
% Flag
IsYPDetected = false;
IsWLDetected = false;
IsPlotting = true;

% Assuming the height of camera is 0.127m
camHeight = 0.127; % meter
focalLen = 0.00304;
SizeOfPixel = 1.12e-6;
resolutionRatio = 8;

% Process above ground and on the ground part seperately
% to detect yellow poles and white lines
[nX, nY, ~] = size(img);

Glevel = 165;   % for 410*308, it's 180
Midlevel = floor((nX - Glevel)/2);


%% Crop the image to increase the speed
HighImg = img(1:Glevel,:,:);
LowImg = img(Glevel+1:end,:,:);


%% Find yellow poles and compute its angle and distance 
% Transform into HSV space and detect yellow poles
HighImgHsv = rgb2hsv(HighImg);

% Make sure yellow pole is detected
HighImgYellow = false(Glevel,nY);
for i = 1 : Glevel  
    for j = 1 : nY  
        hij = HighImgHsv(i, j, 1);  
        sij = HighImgHsv(i, j, 2);  
        vij = HighImgHsv(i, j, 3);  
        
        if ( hij >= 0.11 && hij<= 0.16 ) && ( sij >= 0.3 && sij <= 0.9 ) % && ( vij >= 0.18 && vij <= 1)  
            
            HighImgYellow(i, j) = true;   
        end  
    end  
end

% subplot(221); imshow(HighImgYellow);
% subplot(222); imshow(HighImgHsv(:,:,1))
% subplot(223); imshow(HighImgHsv(:,:,2))
% subplot(224); imshow(HighImgHsv(:,:,3))

% Binarize image
HighBWHsv = imbinarize(HighImgHsv(:,:,2),0.5);

% Find the intersect of HSV and yellow area
HighBWHsv = bitand(HighBWHsv, HighImgYellow);
HighBWBiggestObj = bwareafilt(HighBWHsv,1); % find the biggest object

% Get the position of the middle bottom point of yellow pole
HighBWCanny = edge(HighBWBiggestObj, 'canny');

if sum(sum(HighBWHsv,1)) > 0 && bwarea(HighBWBiggestObj) > 30
    

    SumCanny = sum(HighBWCanny,2);
    YPTop = find(SumCanny>0, 1, 'first'); % Find the top index
    YPBottom = find(SumCanny>0, 1, 'last'); % Find the bottom index

    SumCanny = sum(HighBWCanny,1);
    YPLeft = find(SumCanny>0, 1, 'first'); % Find the left index
    YPRight = find(SumCanny>0, 1, 'last'); % Find the right index

    YPPoint = [YPBottom, (YPLeft + YPRight)/2];

    % Calculate the angle between yellow and image center
    FOV = 60; % Assuming FOV is 60 deg
    YPAng = (YPPoint(2) - nY/2)/nY*FOV;

    % Calculate the distance between yellow pole and camera
    rowDiff = abs(YPPoint(1) - nX/2);
    YPDist = (camHeight * focalLen)/(rowDiff * SizeOfPixel * resolutionRatio);
    
    IsYPDetected = true;
    
else
    
    IsYPDetected = false;
    
    YPPoint = [-1,-1];
    YPAng = -1;
    YPDist = -1;
    
end




%% Find white lines and compute its angle and Distance
% LowImg = histeq(LowImg);
LowHsv = rgb2hsv(LowImg);

% This method is easily got tricked by light
LowHsvWhite = zeros(nX - Glevel,nY);
for i = 1 : (nX - Glevel)  
    for j = 1 : nY  
        hij = LowHsv(i, j, 1);  
        sij = LowHsv(i, j, 2);  
        vij = LowHsv(i, j, 3);  
        
        if  ( sij >= 0 && sij <= 0.3 ) && ( vij >= 0.6 && vij <= 1) % ( hij >= 0.11 && hij<= 0.16 ) &&
            
            LowHsvWhite(i, j) = true;   % logical value
        end  
    end  
end

% subplot(221); imshow(LowImg);
% subplot(222); imshow(LowHsv(:,:,1))
% subplot(223); imshow(LowHsv(:,:,2))
% subplot(224); imshow(LowHsv(:,:,3))



% Binarize image
gray = rgb2gray(LowImg);
T = adaptthresh(gray, 0.6);
LowBW = imbinarize(gray, T);

% Filter out non white part
LowBW = bitand(LowBW, LowHsvWhite);

if sum(sum(LowBW,1)) > 0 
    
    IsWLDetected = true;

    
    LowBWOpen = LowBW;
    % Use open function to remove noise
    se = strel('disk', 3);
    LowBWOpen = imopen(LowBW, se);

    % new = LowBWOpen;
    % 
    % CC = bwconncomp(LowBWOpen);
    % numPixels = cellfun(@numel,CC.PixelIdxList);
    % [biggest,idx] = max(numPixels);
    % LowBWOpen(CC.PixelIdxList{idx}) = 0;
    % LowBWOpen = imsubtract(new, LowBWOpen);


    %% Detect edges
    LowBWCanny = edge(LowBWOpen,'Prewitt'); % Detect edges using Canny


    %% Detect lines using hough transform
    [H,T,R] = hough(LowBWCanny,'RhoResolution',0.5,'ThetaResolution',0.5);

    numpeaks = 7; %Specify the number of peaks
    P  = houghpeaks(H,numpeaks);

%     lines = houghlines(LowBWCanny,T,R,P,'FillGap',70,'MinLength',100);
    lines = houghlines(LowBWCanny,T,R,P,'FillGap',15,'MinLength',70);


    %% Calculate mid point
    if size(lines,2) > 1  % if there is two lines or more
        
        WLLeft = floor((Midlevel - csc(lines(1).theta/180*pi)*lines(1).rho)/(-cot(lines(1).theta/180*pi)));
        WLRight = floor((Midlevel - csc(lines(2).theta/180*pi)*lines(2).rho)/(-cot(lines(2).theta/180*pi)));
        WLMid = (WLLeft+WLRight)/2; 

        % Calculate the angle between white line and y axis
        % -x to +y to x, means -90 to 0 to 90 deg
        WLAng = (lines(1).theta/180*pi + lines(2).theta/180*pi)/2;  % radian
        
        WLPoint = [Midlevel + Glevel, WLMid];

        % Calculate the distance between white line middle point and camera
        rowDiff = abs(WLPoint(1) - nX/2);
        WLDist = (camHeight * focalLen)/(rowDiff * SizeOfPixel * resolutionRatio);

    elseif size(lines,2) == 1
        WLLeft = floor((Midlevel - csc(lines(1).theta/180*pi)*lines(1).rho)/(-cot(lines(1).theta/180*pi)));
        WLRight = WLLeft;
        WLMid = WLLeft;

        WLAng = lines(1).theta/180*pi;
        
        WLPoint = [Midlevel + Glevel, WLMid];

        % Calculate the distance between white line middle point and camera
        rowDiff = abs(WLPoint(1) - nX/2);
        WLDist = (camHeight * focalLen)/(rowDiff * SizeOfPixel * resolutionRatio);

    else
        WLLeft = -1;
        WLRight = -1;
        WLMid = -1;
        WLAng = -1;
        
        WLDist = -1;

    end
    
    

else
    
    IsWLDetected = false;
    
    WLPoint = [-1, -1];
    WLDist = -1;
    WLAng = -1;
    
end


% % Plot
% 
% if ~IsPlotting
%     return
% end
% 
% figure
% subplot(221); imshow(HighImg);
% subplot(222); imshow(HighImgYellow);
% if IsYPDetected
%     subplot(223); imshow(HighBWBiggestObj);
%     subplot(224); imshow(HighBWCanny); 
%     hold on; plot( YPPoint(2), YPPoint(1),'*'); hold off;
% end
% 
% 
% 
% figure
% subplot(221);imshow(LowImg); 
% subplot(222);imshow(LowBW); 
% 
% 
% if IsWLDetected
%     
%     subplot(223);imshow(LowBWOpen); 
%     subplot(224);imshow(LowBWCanny); 
%     hold on
% 
%     % Plot lines on bw image
%     max_len = 0; % record max length
%     for k = 1:length(lines)
%        xy = [lines(k).point1; lines(k).point2];
%        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%        % Plot beginnings and ends of lines
%        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
% 
%        % Determine the endpoints of the longest line segment
%        len = norm(lines(k).point1 - lines(k).point2);
%        if ( len > max_len)
%           max_len = len;
%           xy_long = xy;
%        end
%     end
% 
%     if WLMid > 0
%         plot(WLMid, Midlevel, '*', WLLeft, Midlevel, '*', WLRight, Midlevel, '*' );
%     end
% 
%     hold off
%     
% end
% 
% %     info = [YPDist, YPAng , WLDist, WLAng];
% %     disp(info);
% 
% end
% 
% 
