classdef LightSimulator < handle
% Class used to simulate a near-field non-isotropic disk light source
% which is placed in an environment relative to a frame camera origin
% with a line-scan hyperspectral camera
% Author: Jasprabhjit Mehami, 13446277


    properties(Access = private)
        %Pose of source w.r.t to frame camera
        ligSourLoc
        ligSourOrien
        ligSourDirVec

        maxRadiantInt
        mu
        rAtt
        distFromSrc
        fig
        surfDir
        targetPts
        targetbandInten
        numBands
        numPts
        targetScat
        RIDspline
        realRID
        T_F_2_S
        irrCube
        XCube
        YCube
        ZCube
        cubeS
        numCubes
        nameCubes
        bandCBMax
    end

    methods
        function obj = LightSimulator(varargin)

            %simulator with real RID in figure
            if nargin == 8
                %Constuctor of class
                obj.ligSourLoc = varargin{1};
                obj.ligSourOrien = varargin{2};
                obj.ligSourDirVec = varargin{2}*[0;0;1];
                obj.maxRadiantInt = varargin{3};
                obj.rAtt = varargin{4};
                obj.mu = varargin{5};
                obj.fig = varargin{6};
                obj.surfDir = varargin{7};
                obj.RIDspline = varargin{8};

                obj.realRID = true;

                T_S_2_F = [obj.ligSourOrien, obj.ligSourLoc; 0,0,0,1];
                obj.T_F_2_S = T_S_2_F\eye(4);

                %plot light source and its directional planar slice (slice will
                %not be visible on start up)
                obj.SetupSimulatorFigure();
                obj.PlotLightSource();
                obj.PlotRadiantSlice();

                % Simulator with real measurements of different bands in figure
            elseif nargin == 7
                %Constuctor of class
                obj.ligSourLoc = varargin{1};
                obj.ligSourOrien = varargin{2};
                obj.ligSourDirVec = varargin{2}*[0;0;1];
                obj.fig = varargin{3};
                obj.targetPts = varargin{4};
                obj.targetbandInten = varargin{5};
                obj.numBands = varargin{6};
                obj.numPts = varargin{7};

                obj.PlotLightSource();
                obj.PlotTargetPts();

                % Simulator with Cosine RID without figure
            elseif nargin == 5
                %Constuctor of class
                obj.ligSourLoc = varargin{1};
                obj.ligSourOrien = varargin{2};
                obj.ligSourDirVec = varargin{2}*[0;0;1];
                obj.maxRadiantInt = varargin{3};
                obj.mu = varargin{4};
                obj.rAtt = varargin{5};
                obj.realRID = false;

                T_S_2_F = [obj.ligSourOrien, obj.ligSourLoc; 0,0,0,1];
                obj.T_F_2_S = T_S_2_F\eye(4);
                % simulator no RID in figure to plot light source
            elseif nargin == 3
                %Constuctor of class
                obj.ligSourLoc = varargin{1};
                obj.ligSourOrien = varargin{2};
                obj.ligSourDirVec = varargin{2}*[0;0;1];
                obj.fig = varargin{3};
                obj.realRID = false;
                obj.PlotLightSource();
            else
                error("Wrong number of inputs into class");
            end

            obj.numCubes = 0;
        end

        function PlotTargetPts(obj)
            figure(obj.fig);

            X = obj.targetPts(:,1);
            Y = obj.targetPts(:,2);
            Z = obj.targetPts(:,3);
            intenFirstBand = obj.targetbandInten(:,1);

            obj.targetScat = scatter3(X, Y, Z, 5, intenFirstBand, 'filled'); hold on;

            caxis([0, max(intenFirstBand, [], 'all')])
            colormap(hot(1000)); colorbar;

            %band slider
            slider_band = uicontrol('Parent',obj.fig,'Style','slider','Position',[81,120,300,23],...
                'value',1, 'min', 1, 'max',obj.numBands, 'SliderStep', [1/(obj.numBands+1), 1/(obj.numBands+1)]);
            bgcolor = obj.fig.Color;
            uicontrol('Parent',obj.fig,'Style','text','Position',[50,90,23,23],...
                'String','1','BackgroundColor',bgcolor);
            uicontrol('Parent',obj.fig,'Style','text','Position',[350,90,40,23],...
                'String',num2str(obj.numBands),'BackgroundColor',bgcolor);
            uicontrol('Parent',obj.fig,'Style','text','Position',[150,90,200,23],...
                'String','Band','BackgroundColor',bgcolor);
            %             disp_dist = uicontrol('Parent',obj.fig,'Style','text','Position',[200,145,50,20],...
            %                 'String', num2str(obj.distFromSrc),'BackgroundColor', [1,1,1]);
            %callback function at the end of the script
            slider_band.Callback = @(src, eventData) obj.band_callback(src, eventData);

            drawnow();
        end

        function PlotIrradianceCube(obj, irrCube, XCube, YCube, ZCube, cubeName)
            figure(obj.fig);
            obj.numCubes = obj.numCubes + 1;
            obj.irrCube{obj.numCubes} = irrCube;
            obj.XCube{obj.numCubes} = XCube;
            obj.YCube{obj.numCubes} = YCube;
            obj.ZCube{obj.numCubes} = ZCube;
            obj.nameCubes{obj.numCubes} = cubeName;

            X = XCube;
            Y = YCube;
            Z = ZCube;
            intenFirstBand = irrCube(:,:,1);
            obj.cubeS{obj.numCubes} = surf(X,Y,Z,intenFirstBand, 'EdgeColor','none');hold on;

            dd = uicontrol('Parent',obj.fig,'Style','popupmenu','Position',[81,150,300,23],...
                'String',obj.nameCubes);
            dd.Callback = @(src, event) obj.selection(src, event);
            drawnow();

            

            for i = 1:obj.numBands
                curBandInt = obj.targetbandInten(:,i);
                tempVec = curBandInt(:);

                for j = 1:obj.numCubes
                    curBandInt = obj.irrCube{j};
                    curBandInt = curBandInt(:,:,i);
                    tempVec = [tempVec; curBandInt(:)];
                end

                obj.bandCBMax(i) = max(rmoutliers(tempVec, 'quartiles'), [], 'all');
            end

        end

        % Create ValueChangedFcn callback:
        function selection(obj, src, ~)
            val = src.Value;

            %turn off visibility of all surface plots
            for i = 1:obj.numCubes
                obj.cubeS{i}.Visible = 'off';
            end

            obj.cubeS{val}.Visible = 'on';
        end


        function PlotLightSource(obj)
            %Plot light source and arrow in passed figure
            figure(obj.fig);
            scatter3(obj.ligSourLoc(1), obj.ligSourLoc(2), obj.ligSourLoc(3), 250, [0,1,0], 'filled');
            arrow3(obj.ligSourLoc', (obj.ligSourLoc + 0.5*obj.ligSourDirVec)', 'v', 5);

            poseLightSrc = [obj.ligSourOrien, obj.ligSourLoc; 0,0,0,1];
            trplot(poseLightSrc, 'rviz', 'length', 0.1);

            axis equal; drawnow();
        end

        function PlotRadiantSlice(obj)
            %plot a slice of the light source along the principal axis into
            %the passed in figure

            figure(obj.fig);

            %create slice mesh points in light source c.f
            x = linspace(-0.5, 0.5, 100);
            y = linspace(-0.5, 0.5, 100);
            z = obj.distFromSrc;

            [X, Y, Z] = meshgrid(x,y,z);

            %Transform mesh to frame c.f
            XYZrot = [X(:),Y(:),Z(:)]*obj.ligSourOrien';
            rows = size(X);
            X = reshape(XYZrot(:,1),rows) + obj.ligSourLoc(1);
            Y = reshape(XYZrot(:,2),rows) + obj.ligSourLoc(2);
            Z = reshape(XYZrot(:,3),rows) + obj.ligSourLoc(3);

            %radiant intensity magnitude on mesh
            radIntMag = RadiantIntensityMesh(obj, X, Y, Z);

            %visualise radiant intensity in simulator
            obj.surfDir.XData = X;
            obj.surfDir.YData = Y;
            obj.surfDir.ZData = Z;
            obj.surfDir.CData = radIntMag;
            obj.surfDir.EdgeColor = 'none';

            colormap(hot(100000));
            colorbar; caxis([0, obj.maxRadiantInt]);

            axis equal; drawnow();
        end

        function h = PlotRID3D(obj, numPtsSph)
            % Plot 3D RID in figure.

            if ~exist('numPtsSph', 'var')
                numPtsSph = 100;
            end

            %sphere mesh for plotting RID
            [X,Y,Z] = sphere(numPtsSph);
            [rows, cols] = size(X);

            %compute RID on sphere
            PhiRID = obj.ComputeRID(X(:)',Y(:)',Z(:)');

            % Get azimuth and elevation angles of all points on sphere and
            % use radiant intensity as radius for plotting
            [~, azi, elev] = cart2sphZ(X(:)',Y(:)',Z(:)');
            [xP, yP, zP] =  sphZ2cart(PhiRID, azi, elev);


            xP = reshape(xP,[rows, cols]);
            yP = reshape(yP,[rows, cols]);
            zP = reshape(zP,[rows, cols]);

            h = figure('Name', '3D Radiant Intensity Distribution');

            surf(xP, yP, zP, 'EdgeColor', 'none'); hold on;
            colormap(turbo(numPtsSph/3));
            c = colorbar();
            c.Label.String = 'Radiant Intensity';

            %plot black sphere as light origin
            %             scatter3(0, 0, 0, 50, [0,1,0], 'filled');
            axis equal;
            grid off;

            % plot principal direction arrow
            arrow3([0,0,0],[0,0,max(zP, [], 'all')*1.2] , 'v', 2);
            axis equal;
            axis off;

            drawnow();
        end

        function [PhiRID, r] = ComputeRID(obj, x, y, z)
            % Compute the RID for given points in the coordinate frame of
            % the light source. This will work for both realistic models
            % with measured RID and ideal cosine model.
            %
            % INPUT:
            %       pts - columnwise points in the coordinate frame of the
            %           light source. [3 x N]
            % OUTPUT:
            %       phiRID - radiant intensity at points

            if obj.realRID
                %convert to spherical, ignore radius
                [r, azi, elev] = cart2sphZ(x,y,z);

                %determine azimuth RID from measured RID spline. Assumed to
                %be measurements on the XZ plane.
                ridAzi = fnval(obj.RIDspline, azi);

                %any measurements outside of the azimuth angle pi/2 to -pi/2
                %from z-axis is zero. Ignore any negatives
                ridAzi(azi > pi/2) = 0;
                ridAzi(azi < -pi/2) = 0;
                ridAzi(ridAzi < 0) = 0;

                %elevation RID from cosine model. Highest at the top and
                %decays towards +/-pi/2. obj.mu will control the amount of
                %decay or directionality.
                ridElev = cos(elev).^obj.mu;

                %final RID is a multiplication of both RID components and
                %scaled using maxRadiantInt
                PhiRID = obj.maxRadiantInt .* ridAzi .* ridElev;

                return;
            end

            %purely cosine RID
            [r, theta] = cart2rtlightsrc(x,y,z);

            PhiRID = obj.maxRadiantInt .* (cos(theta).^obj.mu);
        end

        function PlotHemisphereMesh(obj, noPtsHem, orien, locCentre, radius, S)
            %INPUTS:
            %       noPtsHem - number of points used for hemisphere mesh
            %       orien - rotation of hemisphere relative to frame camera
            %       locCentre -  centre of hemisphere relative to frame camera
            %       radius - radius of hemisphere
            %       S - surface mesh object

            %create unit hemisphere mesh
            [X,Y,Z] = sphere(noPtsHem);
            [rows, ~] = size(Z);

            %scale hemisphere using radius
            X = radius.*X((rows-1)/2:end,:);
            Y = radius.*Y((rows-1)/2:end,:);
            Z = radius.*Z((rows-1)/2:end,:);

            %rotate
            XYZrot = [X(:),Y(:),Z(:)]*orien;

            rows_cols = size(X);

            %translate
            X = reshape(XYZrot(:,1),rows_cols) + locCentre(1);
            Y = reshape(XYZrot(:,2),rows_cols) + locCentre(2);
            Z = reshape(XYZrot(:,3),rows_cols) + locCentre(3);

            radianceInSphere = zeros(size(X));

            for i = 1:rows_cols(1)
                for j = 1:rows_cols(2)
                    pntSph = [X(i,j); Y(i,j); Z(i,j)];

                    %calculate normal
                    normSph = pntSph - locCentre';
                    %normalise vector to get direction vector
                    normSph = normSph/norm(normSph);

                    radianceInSphere(i,j) = RadianceInMaterialPoint(obj, pntSph, normSph);
                end
            end

            %             radIntMag = RadiantIntensityMesh(obj, X, Y, Z);

            figure(obj.fig);

            S.XData = X;
            S.YData = Y;
            S.ZData = Z;
            S.CData = radianceInSphere;
            %             obj.surfDir.Visible = 'on';
            S.EdgeColor = 'none';


            axis equal; drawnow();
        end

        function [radIntMag, radIntVec] = RadiantIntensityMesh(obj, X, Y, Z)
            %calculate the radiant intensity of the light source for a
            %given mesh of points. The points should be w.r.t to the frame
            %camera coordinate frame.
            [rows, cols] = size(X);

            pts = [X(:),Y(:),Z(:)]';


            S_P =  obj.NonIsotropicDiskLightModelRID(pts);
            radIntMag = vecnorm(S_P, 2, 1);
            radIntMag = reshape(radIntMag(1,:),[rows, cols]);
            radIntVec = reshape(S_P, [rows, cols, 3]);
        end

        function [radIntMag, radIntVec] = RadiantIntensityAtPoint(obj, pnt)
            %calculate the radiant intensity at a given point from the
            %source

            radIntVec =  obj.NonIsotropicDiskLightModelRID(pnt);

            if isvector(radIntVec)
                radIntMag = norm(radIntVec);
            else
                radIntMag = vecnorm(radIntVec,2,1);
            end
        end

        function RadiIn = RadianceInMaterialPoint(obj, pnt, normal)
            %calculate the in going radiance from the source to some point
            %on the material given its normal and location.

            %Radiant intensity at point
            [~, radIntVec] = RadiantIntensityAtPoint(obj, pnt);

            %Radiance received at point on surface
            RadiIn = dot(-radIntVec, normal);

            %incoming radiance cannot be negative.
            if RadiIn < 0
                RadiIn = 0;
            end
        end

        function RadiOut = RadianceOutMaterialPoint(obj, pnt, normal, reflectance)
            %calculate the out going radiance that is reflected/emitted
            %at a point on a surface with a given normal and reflectance
            %which receives incoming radiance from the source.

            %Radiant intensity at point
            [~, radIntVec] = RadiantIntensityAtPoint(obj, pnt);

            %Radiance received at point on surface
            RadiIn = dot(-radIntVec, normal);

            if RadiIn < 0
                RadiIn = 0;
            end

            %Radiance outgoing at point on surface
            RadiOut = (reflectance/pi) * RadiIn;

        end

        function lightDir = get_SourDirVec(obj)
            lightDir = obj.ligSourDirVec;
            return
        end

        function loc = get_ligSourLoc(obj)
            loc = obj.ligSourLoc;
            return;
        end

        function band_callback(obj, src, ~)
            curBand = round(src.Value);
            curBandInt = obj.targetbandInten(:,curBand);
            obj.targetScat.CData = curBandInt;
            
            for i = 1:obj.numCubes
                curBandInt = obj.irrCube{i};
                obj.cubeS{i}.CData = curBandInt(:,:,curBand);
            end

            caxis([0, obj.bandCBMax(curBand)])
        end


        function distFromSrc_callback(obj, src, ~, texboxHandle)
            obj.distFromSrc = src.Value;
            obj.PlotRadiantSlice();
            texboxHandle.String = num2str(src.Value);
        end

        function mu_callback(obj, src, ~, texboxHandle)
            obj.mu = src.Value;
            obj.PlotRadiantSlice();
            texboxHandle.String = num2str(src.Value);
        end

        function surfDir_visible_callback(obj, src, ~)
            value = get(src,'Value');

            if value
                obj.surfDir.Visible = 'on';
            else
                obj.surfDir.Visible = 'off';
            end
        end


        function SetupSimulatorFigure(obj)
            figure(obj.fig);

            obj.distFromSrc = 0.2;

            %distFromSrc slider
            slider_dist = uicontrol('Parent',obj.fig,'Style','slider','Position',[81,120,300,23],...
                'value',obj.distFromSrc, 'min', 0, 'max',1, 'SliderStep', [0.1/1, 0.1/1]);
            bgcolor = obj.fig.Color;
            uicontrol('Parent',obj.fig,'Style','text','Position',[50,90,23,23],...
                'String','0','BackgroundColor',bgcolor);
            uicontrol('Parent',obj.fig,'Style','text','Position',[350,90,23,23],...
                'String','1','BackgroundColor',bgcolor);
            uicontrol('Parent',obj.fig,'Style','text','Position',[150,90,200,23],...
                'String','Distance from the source','BackgroundColor',bgcolor);
            disp_dist = uicontrol('Parent',obj.fig,'Style','text','Position',[200,145,50,20],...
                'String', num2str(obj.distFromSrc),'BackgroundColor', [1,1,1]);
            %callback function at the end of the script
            slider_dist.Callback = @(src, eventData) obj.distFromSrc_callback(src, eventData, disp_dist);

            %mu slider
            slider_mu = uicontrol('Parent',obj.fig,'Style','slider','Position',[81,200,300,23],...
                'value', obj.mu, 'min',0, 'max',10, 'SliderStep', [0.5/10, 0.5/10]);
            bgcolor = obj.fig.Color;
            uicontrol('Parent',obj.fig,'Style','text','Position',[50,170,23,23],...
                'String','0','BackgroundColor',bgcolor);
            uicontrol('Parent',obj.fig,'Style','text','Position',[350,170,23,23],...
                'String','10','BackgroundColor',bgcolor);
            uicontrol('Parent',obj.fig,'Style','text','Position',[150,170,200,23],...
                'String','mu','BackgroundColor',bgcolor);
            disp_mu = uicontrol('Parent',obj.fig,'Style','text','Position',[200,225,50,20],...
                'String', num2str(obj.mu),'BackgroundColor', [1,1,1]);
            %callback function at the end of the script
            slider_mu.Callback = @(src, eventData) obj.mu_callback(src, eventData, disp_mu);

            %checkerbox to turn surface plane visibility on/off
            surfDirDispCheck = uicontrol('Parent',obj.fig,'Style','checkbox', 'String', 'Display Direction Plane', 'Position', [20,45,200,20] );
            surfDirDispCheck.Callback = @(src, eventData) obj.surfDir_visible_callback(src, eventData);

        end
    end

    methods (Access = private)
        function S_P = NonIsotropicDiskLightModelRID(obj, pntF)
            %The model for a non-istotropic near field disk light model which
            %includes a real RID and light attenutation modelled by an equation which
            %does not contain a singularity.
            %Inputs:
            %       Phi0 - maximum radiant intensity along the principal direction
            %       mu - parameter which alters the shape of the RID with
            %            the elevation angle
            %       pnt - 3D points w.r.t to frame camera
            %       rD - effective radius of the disk which alters the shape of the
            %            attenuation drop-off with distance
            %       ridSpline - spline model of the RID function w.r.t to
            %            azimuth angle
            %       T_F_2_S - Transformation from the frame camera to the
            %                 light source
            %Output:
            %       S_P - radiant intensity from the source to the
            %             point-of-interest along the light vector.

            %transform point to light source c.f
            pntS = obj.T_F_2_S*[pntF;ones(1, size(pntF,2))];

            %computer RID
            [phiRID, r] = obj.ComputeRID(pntS(1,:), pntS(2,:), pntS(3,:));

            %light vector. This is actually defined as starting from the
            %surface point going to the light source, but when simulating
            %the other direction is useful.
            l = pntF - obj.ligSourLoc;

            %normalise light vector (direction vector of l)
            dir_l = l./r;

            %attenuation function
            f_att = 1./((r./obj.rAtt + 1).^2);

            %radiant intensity vector from the source to the
            %point-of-interest.
            S_P = phiRID .* f_att .* dir_l;
        end
    end
end
