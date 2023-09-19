function targetSampleNormRadiance = measurecameraintensityontarget(noSamples, targetSamplePtsFrame, targetSampleSurfNorm, lightSrc, target, intNoiseSigma)
%stores the intensity measured at a specific 3D location. The measured
%intensity at the symmetric target would be the exact same
targetSampleNormRadiance = cell(1, noSamples);

for sampleLoop = 1:noSamples
    targetPtsFrame = targetSamplePtsFrame{sampleLoop};
    targetNormal = targetSampleSurfNorm(:, sampleLoop);
    
    %intensity measured at the 3D location by the line-scan camera in the
    %coordinate frame of the frame camera.
    %***** NOTE: Darkening effects, vignetting, sensor irregularties have
    %not been considered yet
    
    numPts = size(targetPtsFrame, 2);
    
    targetInten = zeros(1,numPts);
    
    for pt = 1:numPts
        targetInten(pt) = lightSrc.RadianceOutMaterialPoint(targetPtsFrame(:,pt), targetNormal, target.Reflectance);
    end
    

    %add gaussian white noise to target pixel measurements
    targetIntenNoise = targetInten + normrnd(0, intNoiseSigma, size(targetInten));
        
    targetSampleNormRadiance(sampleLoop) = {targetIntenNoise};
end
end