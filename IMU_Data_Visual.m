function stm32_swv_tcp_reader()
% STM32_SWV_TCP_READER
% Reads MPU6050 accel/gyro values printed over SWV/ITM via printf,
% through an OpenOCD SWO-to-TCP bridge. Plots each sensor in its own
% figure and writes all captured samples to a CSV file whenever the
% script stops (Stop button, Ctrl+C, error, or normal exit).
%
% Expects firmware printf lines of the form (plain integers, no decimal
% point -- values are milli-g / milli-deg-per-second):
%   ACCEL X: %ld, Y: %ld, Z: %ld | GYRO X: %ld, Y: %ld, Z: %ld\r\n
%
% Assumes printf is retargeted to ITM_SendChar on the ITM stimulus port
% given by targetStimPort below (commonly port 0).
%
% NOTE: exact OpenOCD "tpiu config" syntax is version-dependent; verify
% against your OpenOCD version (run "tpiu config" with no args in the
% OpenOCD console) before relying on the command line you use to start it.

%% --- User settings ---
close all
host = '127.0.0.1';
port = 3443;
targetStimPort = 0;     % ITM stimulus port printf/ITM_SendChar writes to
readTimeout = 5;        % seconds, for the initial TCP connection
rollingWindow = 300;    % samples shown on the live plots (CSV keeps everything)
csvFilename = fullfile(pwd, 'imu_log.csv');
autosaveEvery = 100;    % write the CSV every N samples as a safety net
milliToUnit = 1000;     % firmware sends milli-g / milli-dps as integers

%% --- Attitude estimation settings ---
% Which raw sensor axis reads ~1g at rest (i.e. which one is "up" in the
% sensor's own frame). Earlier calibration on this board found X to be
% the vertical axis -- change this if your mounting is different.
verticalAxis = 'X';        % 'X', 'Y', or 'Z'
compFilterAlpha = 0.98;    % weight on gyro integration vs accel correction
                           % (closer to 1 = trusts gyro more / drifts less
                           % on quick moves, closer to 0 = trusts accel
                           % more / noisier but no long-term tilt drift)

%% --- Data storage (grows for the whole session; saved to CSV at the end) ---
sampleIdx = 0;
accX = []; accY = []; accZ = [];
gyroX = []; gyroY = []; gyroZ = [];
timestamps = [];

%% --- Attitude filter state ---
roll = 0; pitch = 0; yaw = 0;    % radians
attitudeInitialized = false;
attitudeTimerId = [];

%% --- Connect ---
try
    t = tcpclient(host, port, "Timeout", readTimeout);
    disp("Connected to OpenOCD SWO TCP server.");
catch ME
    error("Could not connect to %s:%d. Is OpenOCD running with a tpiu TCP server on this port? Details: %s", ...
        host, port, ME.message);
end

% onCleanup fires when this function's workspace is torn down, including
% on Ctrl+C, so the CSV gets written no matter how the script stops.
% This only works reliably because saveCSV is a NESTED function (shares
% this function's workspace) rather than a separate local function.
cleanupObj = onCleanup(@saveCSV); %#ok<NASGU>

%% --- Line parsing setup ---
% Plain integers only -- no decimal points. If your firmware later goes
% back to printing decimals, this pattern needs "-?\d+\.?\d*" instead,
% but only once the firmware side is actually printing valid numbers.
lineBuffer = '';
pattern = ['ACCEL X:\s*(-?\d+),\s*Y:\s*(-?\d+),\s*Z:\s*(-?\d+)\s*\|\s*' ...
           'GYRO X:\s*(-?\d+),\s*Y:\s*(-?\d+),\s*Z:\s*(-?\d+)'];

%% --- Figures: one per sensor ---
stopRequested = false;

figAccel = figure('Name', 'Accelerometer');
axAccel = axes(figAccel); hold(axAccel, 'on'); grid(axAccel, 'on');
title(axAccel, 'MPU6050 Accelerometer'); xlabel(axAccel, 'Sample'); ylabel(axAccel, 'g');
hAx = plot(axAccel, nan, nan, '-', 'DisplayName', 'X');
hAy = plot(axAccel, nan, nan, '-', 'DisplayName', 'Y');
hAz = plot(axAccel, nan, nan, '-', 'DisplayName', 'Z');
legend(axAccel);

% Preferred way to stop: this button exits the read loop normally, so
% onCleanup/saveCSV runs cleanly with no interrupt involved. Ctrl+C still
% works as a fallback, but isn't guaranteed to save cleanly every time.
uicontrol(figAccel, 'Style', 'pushbutton', 'String', 'Stop & Save', ...
    'Units', 'normalized', 'Position', [0.02 0.02 0.15 0.06], ...
    'Callback', @stopButtonCallback);

figGyro = figure('Name', 'Gyroscope');
axGyro = axes(figGyro); hold(axGyro, 'on'); grid(axGyro, 'on');
title(axGyro, 'MPU6050 Gyroscope'); xlabel(axGyro, 'Sample'); ylabel(axGyro, 'deg/s');
hGx = plot(axGyro, nan, nan, '-', 'DisplayName', 'X');
hGy = plot(axGyro, nan, nan, '-', 'DisplayName', 'Y');
hGz = plot(axGyro, nan, nan, '-', 'DisplayName', 'Z');
legend(axGyro);

%% --- 3D attitude visualization (drone frame) ---
figAttitude = figure('Name', 'Attitude');
axAttitude = axes(figAttitude);
hold(axAttitude, 'on'); grid(axAttitude, 'on'); axis(axAttitude, 'equal');
xlim(axAttitude, [-1.5 1.5]); ylim(axAttitude, [-1.5 1.5]); zlim(axAttitude, [-1.5 1.5]);
xlabel(axAttitude, 'X'); ylabel(axAttitude, 'Y'); zlabel(axAttitude, 'Z');
title(axAttitude, 'Estimated Attitude');
view(axAttitude, 3);

% Fixed world-frame reference tripod (does NOT rotate) so you can see
% the drone's orientation relative to a stationary reference.
plot3(axAttitude, [0 1.3], [0 0], [0 0], 'r-', 'LineWidth', 1);
plot3(axAttitude, [0 0], [0 1.3], [0 0], 'g-', 'LineWidth', 1);
plot3(axAttitude, [0 0], [0 0], [0 1.3], 'b-', 'LineWidth', 1);
text(axAttitude, 1.35, 0, 0, 'X'); text(axAttitude, 0, 1.35, 0, 'Y'); text(axAttitude, 0, 0, 1.35, 'Z');

droneTransform = hgtransform('Parent', axAttitude);
buildDroneShape(droneTransform);

attitudeText = text(axAttitude, -1.4, -1.4, 1.3, '', 'FontName', 'FixedWidth');

uicontrol(figAttitude, 'Style', 'pushbutton', 'String', 'Reset Yaw', ...
    'Units', 'normalized', 'Position', [0.02 0.02 0.15 0.06], ...
    'Callback', @resetYawCallback);

%% --- Live read + decode + parse loop ---
buffer = uint8.empty;
disp("Reading SWO stream. Click 'Stop & Save' to stop cleanly (or Ctrl+C as a fallback).");

while ~stopRequested
    if t.NumBytesAvailable > 0
        newBytes = read(t, t.NumBytesAvailable, "uint8");
        buffer = [buffer, newBytes]; %#ok<AGROW>
    end

    [chars, buffer] = decodeSWIT(buffer, targetStimPort);

    if ~isempty(chars)
        lineBuffer = [lineBuffer, char(chars)]; %#ok<AGROW>

        % Split off complete lines (terminated by \n; strip any \r too)
        while true
            nlPos = find(lineBuffer == newline, 1);
            if isempty(nlPos)
                break;
            end
            rawLine = lineBuffer(1:nlPos-1);
            lineBuffer = lineBuffer(nlPos+1:end);
            rawLine = strrep(rawLine, sprintf('\r'), '');
            processLine(rawLine);
        end
    end

    pause(0.01);
end

%% --- Nested functions (these share this function's workspace) ---
    function processLine(lineStr)
        tokens = regexp(lineStr, pattern, 'tokens', 'once');
        if isempty(tokens)
            return; % not a matching line (e.g. a boot/log message) -- skip it
        end

        vals = str2double(tokens);
        if any(isnan(vals))
            return; % malformed/partial line -- skip it
        end

        sampleIdx = sampleIdx + 1;
        timestamps(end+1) = sampleIdx; %#ok<AGROW>

        accX(end+1) = vals(1) / milliToUnit; %#ok<AGROW>
        accY(end+1) = vals(2) / milliToUnit; %#ok<AGROW>
        accZ(end+1) = vals(3) / milliToUnit; %#ok<AGROW>
        gyroX(end+1) = vals(4) / milliToUnit; %#ok<AGROW>
        gyroY(end+1) = vals(5) / milliToUnit; %#ok<AGROW>
        gyroZ(end+1) = vals(6) / milliToUnit; %#ok<AGROW>

        updateAttitude(accX(end), accY(end), accZ(end), gyroX(end), gyroY(end), gyroZ(end));
        updatePlots();

        if mod(sampleIdx, autosaveEvery) == 0
            saveCSV();   % periodic safety-net save; overwrites csvFilename
        end
    end

    function stopButtonCallback(~, ~)
        stopRequested = true;
    end

    function resetYawCallback(~, ~)
        yaw = 0;
    end

    function [cX, cY, cZ] = remapToVertical(rawX, rawY, rawZ)
        % Reorders the three raw sensor-frame components so that cZ is
        % always the component along whichever physical axis is "up"
        % (verticalAxis), and cX/cY are the other two, in their original
        % relative order. This lets the standard roll/pitch formulas
        % below (which assume Z is vertical) work regardless of how the
        % board is actually mounted.
        switch upper(verticalAxis)
            case 'X'
                cX = rawY; cY = rawZ; cZ = rawX;
            case 'Y'
                cX = rawZ; cY = rawX; cZ = rawY;
            otherwise % 'Z'
                cX = rawX; cY = rawY; cZ = rawZ;
        end
    end

    function updateAttitude(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps)
        if isempty(attitudeTimerId)
            attitudeTimerId = tic;
            dt = 0;
        else
            dt = toc(attitudeTimerId);
            attitudeTimerId = tic;
        end

        [cAx, cAy, cAz] = remapToVertical(ax_g, ay_g, az_g);
        [cGx, cGy, cGz] = remapToVertical(gx_dps, gy_dps, gz_dps);

        % Roll/pitch purely from gravity direction (no drift, but noisy
        % and only valid when the vehicle isn't accelerating much).
        rollAcc  = atan2(cAy, cAz);
        pitchAcc = atan2(-cAx, sqrt(cAy^2 + cAz^2));

        gRollRate  = deg2rad(cGx);
        gPitchRate = deg2rad(cGy);
        gYawRate   = deg2rad(cGz);

        if ~attitudeInitialized
            % Seed directly from the accelerometer instead of filtering
            % up from zero, so the drone doesn't visibly "snap" at startup.
            roll = rollAcc;
            pitch = pitchAcc;
            yaw = 0;
            attitudeInitialized = true;
        else
            roll  = compFilterAlpha * (roll + gRollRate * dt)  + (1 - compFilterAlpha) * rollAcc;
            pitch = compFilterAlpha * (pitch + gPitchRate * dt) + (1 - compFilterAlpha) * pitchAcc;
            yaw   = yaw + gYawRate * dt;   % no absolute reference -- will drift; use Reset Yaw
        end

        updateDroneVisualization();
    end

    function updateDroneVisualization()
        R = makehgtform('zrotate', yaw, 'yrotate', pitch, 'xrotate', roll);
        set(droneTransform, 'Matrix', R);
        set(attitudeText, 'String', sprintf('Roll:  %6.1f deg\nPitch: %6.1f deg\nYaw:   %6.1f deg (drifts, no mag ref)', ...
            rad2deg(roll), rad2deg(pitch), rad2deg(yaw)));
        drawnow limitrate;
    end

    function buildDroneShape(parentTransform)
        % A simple stylized quadcopter frame: a small body cube, four
        % arms in an X pattern, and a rotor ring at each arm tip.
        bodySize = 0.15;
        armLength = 1.0;
        rotorRadius = 0.25;

        % Body: small cube centered at the origin
        [bx, by, bz] = ndgrid([-1 1] * bodySize);
        vertsBody = [bx(:) by(:) bz(:)];
        facesBody = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
        patch('Parent', parentTransform, 'Vertices', vertsBody, 'Faces', facesBody, ...
            'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k');

        % Arms: four diagonal lines from center to arm tips
        armAngles = deg2rad([45 135 225 315]);
        armTips = [armLength * cos(armAngles); armLength * sin(armAngles); zeros(1, 4)];
        colors = {[0.8 0.1 0.1], [0.1 0.6 0.1], [0.1 0.1 0.8], [0.8 0.6 0.1]};

        theta = linspace(0, 2*pi, 17);
        for k = 1:4
            tip = armTips(:, k);
            line('Parent', parentTransform, 'XData', [0 tip(1)], 'YData', [0 tip(2)], ...
                'ZData', [0 tip(3)], 'Color', 'k', 'LineWidth', 2);

            % Rotor ring, drawn flat in the body's local XY plane at the arm tip
            ringX = tip(1) + rotorRadius * cos(theta);
            ringY = tip(2) + rotorRadius * sin(theta);
            ringZ = tip(3) + zeros(size(theta));
            patch('Parent', parentTransform, 'XData', ringX, 'YData', ringY, 'ZData', ringZ, ...
                'FaceColor', colors{k}, 'FaceAlpha', 0.5, 'EdgeColor', colors{k});
        end
    end

    function updatePlots()
        n = numel(timestamps);
        idxStart = max(1, n - rollingWindow + 1);
        idx = idxStart:n;

        set(hAx, 'XData', timestamps(idx), 'YData', accX(idx));
        set(hAy, 'XData', timestamps(idx), 'YData', accY(idx));
        set(hAz, 'XData', timestamps(idx), 'YData', accZ(idx));

        set(hGx, 'XData', timestamps(idx), 'YData', gyroX(idx));
        set(hGy, 'XData', timestamps(idx), 'YData', gyroY(idx));
        set(hGz, 'XData', timestamps(idx), 'YData', gyroZ(idx));

        drawnow limitrate;
    end

    function saveCSV()
        if isempty(timestamps)
            disp('No samples captured -- nothing to save.');
            return;
        end
        T = table(timestamps', accX', accY', accZ', gyroX', gyroY', gyroZ', ...
            'VariableNames', {'Sample','AccelX_g','AccelY_g','AccelZ_g','GyroX_dps','GyroY_dps','GyroZ_dps'});
        writetable(T, csvFilename);
        fprintf('Saved %d samples to %s\n', numel(timestamps), csvFilename);
    end

    function [chars, remaining] = decodeSWIT(buf, wantedPort)
        % Same SWIT packet framing as before (see ARM CoreSight ITM
        % architecture), returning raw payload bytes as characters in
        % packet order since this stream carries ASCII text.
        chars = uint8.empty;
        i = 1;
        n2 = numel(buf);

        while i <= n2
            header = buf(i);
            isSWIT   = bitget(header, 3) == 0;   % bit 2 == 0 -> software source
            sizeCode = bitand(header, 3);         % bits [1:0]
            stimPort = bitshift(header, -3);      % bits [7:3]

            if ~isSWIT || sizeCode == 0 || stimPort ~= wantedPort
                i = i + 1;   % not a recognized packet for our port, skip a byte
                continue;
            end

            switch sizeCode
                case 1, payloadLen = 1;
                case 2, payloadLen = 2;
                case 3, payloadLen = 4;
            end

            if i + payloadLen > n2
                break;   % incomplete packet, wait for more bytes next read
            end

            chars = [chars, buf(i+1 : i+payloadLen)]; %#ok<AGROW>
            i = i + 1 + payloadLen;
        end

        remaining = buf(i:end);
    end

end