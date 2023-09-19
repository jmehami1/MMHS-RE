function [gpTrainingdata, downSamplingGP, radIntGP, radIntGPZero, radVar, diffRad] = solve_gp(targetTrials, locLightSrc, target, T_S_2_F, testingX, gp_model_type, intNoiseSigma, rows, radIntMagGroundtruth)
targetL = targetTrials{end,1};
targetPntFrame = targetTrials{end,3};
targetPntNormals = targetTrials{end,2};

%calculate the direction light vector (point to light source)
pntLightVec = locLightSrc - targetPntFrame;
dirPntLightVec = pntLightVec./vecnorm(pntLightVec);

%Calculate radiant intensity magnitude used for building model
radIntMagPnt = (targetL.*pi)./(target.Reflectance.*dot(targetPntNormals, dirPntLightVec,1));

%find the point w.r.t to light source c.f
targetPntFrameHom = [targetPntFrame; ones(1, length(targetPntFrame(1,:)))];
targetPntLightSrcHom = T_S_2_F\targetPntFrameHom;
targetPntLightSrc = targetPntLightSrcHom(1:3, :);

[ptsRadius,ptsTheta] = cart2sphZ(targetPntLightSrc(1,:), targetPntLightSrc(2,:), targetPntLightSrc(3,:));
gpTrainingdata = [ptsRadius', ptsTheta', radIntMagPnt'];
downSamplingGP = 30;

%% Building model with GP Zero-mean non-symmetric

%Build zero-mean GP model
[mu, varMu] = LightSrcOptmGP(testingX, gpTrainingdata, gp_model_type, intNoiseSigma, downSamplingGP, 10000, false);

radIntGP = reshape(mu,rows);
radIntGPZero = radIntGP;
radVar = reshape(varMu,rows);
diffRad = abs(radIntGP - radIntMagGroundtruth);
end