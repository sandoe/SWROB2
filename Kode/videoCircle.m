function [circleFound, circleColor, confidence] = detectColoredCircle(img)
    % Konverter billedet til gråskala
    grayImg = rgb2gray(img);

    % Brug morphologisk erosion og derefter dilation til at forbedre billedbehandling, valgfrit
    se = strel('disk', 5);
    imgProcessed = imdilate(imerode(grayImg, se), se);

    % Find cirkler i billedet
    [centers, radii, metric] = imfindcircles(imgProcessed, [10 100], 'ObjectPolarity', 'dark', 'Sensitivity', 0.85);

    % Initialiser output variabler
    circleFound = false;
    circleColor = '';
    confidence = 0;

    % Tjek om der blev fundet nogen cirkler
    if ~isempty(centers)
        circleFound = true;
        confidence = max(metric);  % Maksimale confidence værdi fra detektionerne

        % Antag at den cirkel med højeste confidence er den vi er interesseret i
        bestCircleIndex = find(metric == max(metric), 1);
        bestCircleColor = impixel(img, centers(bestCircleIndex,1), centers(bestCircleIndex,2));

        % Definer tærskler for rød og lilla farve
        redThresholdLow = [150, 0, 0];
        redThresholdHigh = [255, 100, 100];
        purpleThresholdLow = [75, 0, 75];
        purpleThresholdHigh = [140, 80, 150];

        % Tjek farve inden for den bedste cirkel
        if all(bestCircleColor(1,:) >= redThresholdLow) & all(bestCircleColor(1,:) <= redThresholdHigh)
            circleColor = 'Red';
        elseif all(bestCircleColor(1,:) >= purpleThresholdLow) & all(bestCircleColor(1,:) <= purpleThresholdHigh)
            circleColor = 'Purple';
        else
            circleColor = 'Other';
        end
    end

    % Visualisér detektionen
    imshow(img);
    hold on;
    viscircles(centers, radii, 'EdgeColor', 'b');
    hold off;
    title(sprintf('Detected Circle: %s (%.2f%% Confidence)', circleColor, confidence * 100));
end

% Initialiser webcam
cam = webcam('/dev/video0');

% Løkke til at fortsætte indtil kriterierne er opfyldt
while true
    img = snapshot(cam);  % Tag et billede med webcam
    [circleFound, circleColor, confidence] = detectColoredCircle(img);  % Analyser billede

    fprintf('Circle found: %s\nCircle Color: %s\nConfidence: %.2f%%\n', ...
        circleFound, circleColor, confidence * 100);

    % Afslut løkke baseret på kriterier
    if circleFound && ((confidence * 100 >= 40 && confidence * 100 <= 80) && (strcmp(circleColor, 'Red') || strcmp(circleColor, 'Purple')))
        disp(confidence*100)
        disp(circleColor)
        disp(circleFound)
        break;
    end
end

% Ryd op
clear('cam');  % Frigør webcam
%clc;  % Ryd kommandovinduet
