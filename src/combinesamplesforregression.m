function targetTrials = combinesamplesforregression(noSamples, yPix, targetSampleNormRadiance, targetSampleSurfNorm, targetSamplePtsFrame)
targetTrials = cell(noSamples, 3);

for trials = 1:noSamples
    targetL = zeros(1,2*trials*yPix);
    targetPntNormals = zeros(3,2*trials*yPix);
    targetPntFrame = zeros(3, 2*trials*yPix);
    
    noPts = 0;
    
    %combines all samples into a single array for the current trials
    for sampleLoop = 1:trials
        noCurPts = length(targetSampleNormRadiance{sampleLoop});
        
        targetL(noPts+1:noPts+noCurPts) = targetSampleNormRadiance{sampleLoop};
        targetPntNormals(:, noPts+1:noPts+noCurPts) = repmat(targetSampleSurfNorm(:, sampleLoop), [1, noCurPts]);
        targetPntFrame(:, noPts+1:noPts+noCurPts) = targetSamplePtsFrame{sampleLoop};

        noPts = noPts + noCurPts;
    end
    
    %clip the arrays to correct size
    targetL = targetL(1:noPts);
    targetPntNormals = targetPntNormals(:, 1:noPts);
    targetPntFrame = targetPntFrame(:, 1:noPts);
    
    targetTrials(trials,:) = {targetL, targetPntNormals, targetPntFrame};    
end
end