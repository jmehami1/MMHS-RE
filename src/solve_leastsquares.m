function [resNormTrials, eigFIM, radIntMagGroundtruth, radIntMagLeastSqr, diffRad] = solve_leastsquares(noSamples, targetTrials, lightSrc, target, intNoiseSigma, locLightSrc, rotLightSrc, XFrame, YFrame, ZFrame)
optOptions = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'SpecifyObjectiveGradient',true, 'CheckGradients', false, ...
    'MaxIterations', 1000000000, 'FunctionTolerance',1e-6, 'MaxFunctionEvaluations',1000000000, 'StepTolerance',1e-6, ...
    'FiniteDifferenceType', 'central', 'ScaleProblem','none');
optOptions.Display = 'none';

optPhiTrials = zeros(noSamples, 3);
resNormTrials = zeros(noSamples, 1);

eigFIM = zeros(noSamples, 3);
rankFIM = zeros(noSamples, 1);

Phi0 = [1, 1, 1];

for trials = 1:noSamples
    
    targetL = targetTrials{trials,1};
    targetPntFrame = targetTrials{trials,3};
    targetPntNormals = targetTrials{trials,2};
    
    [optPhi,resNorm, curRankFIM, curEigFIM] = LightSrcOptmLS(lightSrc, Phi0, targetL, targetPntFrame, targetPntNormals, target.Reflectance, optOptions, intNoiseSigma);
    
    %store optimised parameters and residual
    optPhiTrials(trials, :) = optPhi;
    resNormTrials(trials) = resNorm;
    
    rankFIM(trials) = curRankFIM;
    eigFIM(trials,:) = curEigFIM;
end

lightSrcLeastSqrs = LightSimulator(locLightSrc, rotLightSrc, optPhiTrials(end, 1), optPhiTrials(end, 2), optPhiTrials(end, 3));

%plot the light intensity produced by the light source on the line-scan
%camera's view-plane
radIntMagGroundtruth = lightSrc.RadiantIntensityMesh(XFrame, YFrame, ZFrame);
radIntMagLeastSqr = lightSrcLeastSqrs.RadiantIntensityMesh(XFrame, YFrame, ZFrame);
diffRad = abs(radIntMagGroundtruth - radIntMagLeastSqr);
end