function [targetSamplePtsFrame, targetSampleSurfNorm] = sampletarget(noSamples, maxRange, minRange, yPix, KMat, target, lsIntrinsic, T_LS_2_F, figSim, figViewPlane, maxRadiInt, angleNoise)
%store the points in the frame camera c.f and the normal for each sample
targetSamplePtsFrame = cell(1,noSamples);
targetSampleSurfNorm = zeros(3, noSamples);

for sampleLoop = 1:noSamples
    %get random distance between line-scan and target in metres
    distLS2Target = (maxRange - minRange)*rand() + minRange;
    %plot the light intensity produced by the light source on the line-scan
    
    %pixel location of the centre of target in line-scan image
    pixTargetCent = randi([0,yPix]);
    
    %normalise centre pixel location
    pixTargetCentHom = [0 ; pixTargetCent; 1];
    normTargetCent = KMat\pixTargetCentHom;
    normTargetCent = normTargetCent./normTargetCent(3);
    
    %Use the normalised coordinate to determine the direction of the pixel
    %ray. The translation of the target coordinate frame is found by
    %multiplying the distance from the line-scan camera with the direction
    %of the ray
    dirNormTargetCent = normTargetCent./norm(normTargetCent);
    targetCentPos =  distLS2Target.*dirNormTargetCent;

    
    %pick angle about x-axis between 90deg and 270 deg to get the pose of
    %the target. Only assume rotation about x-axis (yaw only)
    a = deg2rad(-angleNoise);
    b = deg2rad(angleNoise);
    xAngle = pi + (b-a)*rand()+(a);
    %%initially assume a constant normal angle
    rotTargetLS = eul2rotm([0, 0, xAngle], 'ZYX');
    
    %pose of target w.r.t to line-scan camera c.f
    T_target_2_LS = [rotTargetLS, targetCentPos; 0, 0, 0, 1];

    %normal vector of the target surface (z-axis) w.r.t to LS
    normTargetLS = T_target_2_LS(1:3,3);
    
    %left and right edge of the target in its coordinate frame
    targetEdgesPtsHom = [[target.LeftEdge; 1], [target.RightEdge; 1]];
    
    %project target edges to line-scan image
    imgTargetEdges = projectpoints(targetEdgesPtsHom', KMat, T_target_2_LS, [], lsIntrinsic.ImageSize);
    
    %extract v coordinate of pixels
    vImgTargetEdges = imgTargetEdges(:,2);
    
    %Check if the edge pixels are within the size of the line-scan image, else
    %set the edge pixels to be the limits of the line-scan image
    for j = 1:2
        vImgTargetEdges(j) = round(vImgTargetEdges(j));
        
        %outside on the left of the image line
        if vImgTargetEdges(j) < 1
            vImgTargetEdges(j) = 1;
            %outside on the right of the image line
        elseif vImgTargetEdges(j) > yPix
            vImgTargetEdges(j) = yPix;
        end
    end
    
    
    %vector of all pixels that see the target
    if vImgTargetEdges(1) > vImgTargetEdges(2)
        vtargetPix = vImgTargetEdges(2):vImgTargetEdges(1);
    else
        vtargetPix = vImgTargetEdges(1):vImgTargetEdges(2);
    end
    
    %transform to normalized coordinates
    targetPixHom  = [zeros(size(vtargetPix)); vtargetPix; ones(size(vtargetPix))];
    normTargetPixHom = KMat\targetPixHom;
    
    %3D points on target plane
    targetPtsLS = zeros(3, length(vtargetPix));
    ptCount = 0;
    
    %trace pixel ray from the line-scan camera to the target frame to
    %determine its 3D location w.r.t LS
    for pixelLoop = 1:length(vtargetPix)
        pnt = normTargetPixHom(:,pixelLoop);
                
        [pntOnTarget, valid] = camerapixelrayintersectplane(pnt', normTargetLS', targetCentPos');
        
        %ray intersection was not valid
        if ~valid
           continue; 
        end
        
        ptCount = ptCount + 1;
        targetPtsLS(:, ptCount) = pntOnTarget;
    end
    
    targetPtsLS = targetPtsLS(:,1:ptCount);
    
    %transform points from line-scan to frame camera
    targetPtsFrame = T_LS_2_F*[targetPtsLS; ones(1,length(targetPtsLS(1,:)))];
    targetPtsFrame = targetPtsFrame(1:3,:);
    
    %save the target pts in the c.f of the frame camera
    targetSamplePtsFrame(sampleLoop) = {targetPtsFrame};
    
    %pose of target w.r.t frame camera c.f
    T_target_2_F = T_LS_2_F*T_target_2_LS;
    %surface normal of target w.r.t to F
    targetSampleSurfNorm(:,sampleLoop) = T_target_2_F(1:3,3);
    
    
    %plot line and axes of target in the simulator figure
    figure(figSim);
    line(targetPtsFrame(1,:), targetPtsFrame(2,:), targetPtsFrame(3,:), 'Color', [1,0,1], 'LineWidth', 1.5); hold on;
    trplot(T_target_2_F, 'rviz', 'length', (maxRange - minRange)*0.05);
    
    %plot line in the view-plane intensity plot
    figure(figViewPlane);
    line(targetPtsLS(2,:), targetPtsLS(3,:), maxRadiInt.*ones(size(targetPtsLS(2,:))), 'Color', [1,0,1], 'LineWidth', 2);    
end

drawnow();
end