function al5a_teach(serialPort, varargin)
%AL5A_TEACH Interactive Lynxmotion AL5A control via MATLAB Robotics Toolbox.
%   AL5A_TEACH() opens a Robotics Toolbox TEACH window for the Lynxmotion
%   AL5A arm. Joint sliders update a simulation model and (optionally)
%   transmit corresponding servo commands to an SSC-32/SSC-32U controller.
%
%   AL5A_TEACH(SERIALPORT) additionally streams servo pulses to the provided
%   serial port. Pass an empty string or omit the argument to run in
%   simulation mode only.
%
%   AL5A_TEACH(SERIALPORT, 'TravelTime', T, 'BaudRate', B) lets you specify
%   the servo travel time (seconds) and serial baud rate (default 0.75 s and
%   115200 baud). When connected to hardware the function automatically
%   clears the serial port when the figure closes.
%
%   The implementation relies on Peter Corke's Robotics Toolbox for MATLAB
%   (rvctools). Ensure the toolbox is on your MATLAB path prior to running
%   the script.
%
%   Example
%   -------
%   % Launch in simulation only
%   al5a_teach();
%
%   % Connect to hardware on COM3 with a 1 second move time
%   al5a_teach("COM3", 'TravelTime', 1.0);
%
%   See also SerialLink, teach.
%
arguments
    serialPort (1,1) string = ""
end
opts = struct('TravelTime', 0.75, 'BaudRate', 115200);
opts = parse_options(opts, varargin{:});

links = al5a_links();
robot = SerialLink(links, 'name', 'Lynxmotion AL5A', 'base', trotz(pi/2));

% Nominal home configuration (degrees): [base, shoulder, elbow, wrist, gripper]
q0_deg = [0, 15, -40, 25, 0];
q0 = deg2rad(q0_deg);

% Prepare optional serial connection
serialObj = [];
cleanup = [];
if strlength(serialPort) > 0
    serialObj = serialport(serialPort, opts.BaudRate);
    configureTerminator(serialObj, "CR");
    serialObj.Timeout = 0.5;
    cleanup = onCleanup(@()close_serial(serialObj));
end

callback = @(~, q)teach_callback(robot, q, serialObj, opts.TravelTime);
fig = robot.teach(q0, 'deg', false, 'callback', callback, ...
    'qmin', robot.qlim(:,1)', 'qmax', robot.qlim(:,2)');

% Keep MATLAB busy until the window closes to maintain the serial
% connection for the duration of the session.
if ishghandle(fig)
    uiwait(fig);
end

if ~isempty(cleanup)
    delete(cleanup);
end
end

function opts = parse_options(opts, varargin)
    if mod(numel(varargin), 2) ~= 0
        error('Options must be provided as name/value pairs.');
    end
    for idx = 1:2:numel(varargin)
        name = validatestring(varargin{idx}, fieldnames(opts));
        opts.(name) = varargin{idx + 1};
    end
end

function links = al5a_links()
    L = al5a_link_lengths();

    % Define modified DH parameters approximating the AL5A geometry
    links(1) = Link('revolute', 'alpha', pi/2, 'a', 0,         'd', L.base_height, 'offset', 0);
    links(2) = Link('revolute', 'alpha', 0,    'a', L.shoulder, 'd', 0,             'offset', pi/2);
    links(3) = Link('revolute', 'alpha', 0,    'a', L.elbow,    'd', 0,             'offset', 0);
    links(4) = Link('revolute', 'alpha', 0,    'a', L.wrist,    'd', 0,             'offset', 0);
    links(5) = Link('revolute', 'alpha', 0,    'a', 0.050,      'd', 0,             'offset', 0); % gripper rotation

    % Joint limits (radians)
    limits = [
        -pi/2,  pi/2;   % base
        -0.35,  2.0;    % shoulder
        -2.4,   0.35;   % elbow
        -2.0,   2.0;    % wrist
        -1.0,   1.0     % gripper
    ];
    for i = 1:size(limits, 1)
        links(i).qlim = limits(i, :);
    end
end

function lengths = al5a_link_lengths()
    lengths = struct(...
        'base_height', 0.070, ...
        'shoulder', 0.09525, ...
        'elbow', 0.10795, ...
        'wrist', 0.082 ...
    );
end

function teach_callback(robot, q, serialObj, travelTime)
    if isempty(serialObj) || ~isvalid(serialObj)
        return;
    end
    pulses = joints_to_pulses(q(:));
    cmd = format_ssc32_command(pulses, travelTime);
    writeline(serialObj, cmd);

    feedbackAngles = read_feedback_angles(serialObj);
    if ~isempty(feedbackAngles)
        robot.animate(feedbackAngles(:)');
    end
end

function angles = read_feedback_angles(serialObj)
    persistent feedbackWarned
    angles = [];
    if isempty(serialObj) || ~isvalid(serialObj)
        return;
    end

    try
        writeline(serialObj, "QP");
        response = strtrim(readline(serialObj));
    catch err
        if isempty(feedbackWarned) || ~feedbackWarned
            warning('al5a_teach:feedback', ...
                'Unable to read servo feedback: %s', err.message);
            feedbackWarned = true;
        end
        return;
    end

    if strlength(response) == 0
        if isempty(feedbackWarned) || ~feedbackWarned
            warning('al5a_teach:feedback', ...
                'Controller did not return servo feedback data.');
            feedbackWarned = true;
        end
        return;
    end

    values = str2double(split(response));
    if any(isnan(values))
        if isempty(feedbackWarned) || ~feedbackWarned
            warning('al5a_teach:feedback', ...
                'Unexpected servo feedback payload: %s', response);
            feedbackWarned = true;
        end
        return;
    end

    angles = pulses_to_joints(values(:)');
    if ~isempty(angles)
        feedbackWarned = false;
    end
end

function pulses = joints_to_pulses(q)
    configs = servo_configs();
    numChannels = min(numel(q), numel(configs));
    pulses = zeros(numChannels, 1);
    for i = 1:numChannels
        cfg = configs(i);
        angle = max(cfg.min_angle, min(cfg.max_angle, q(i)));
        proportion = (angle - cfg.min_angle) / (cfg.max_angle - cfg.min_angle);
        pulses(i) = round(cfg.min_pulse + proportion * (cfg.max_pulse - cfg.min_pulse));
    end
end

function angles = pulses_to_joints(pulses)
    configs = servo_configs();
    channels = servo_channels();
    numServos = min(numel(configs), numel(channels));
    if numel(pulses) < max(channels(1:numServos)) + 1
        angles = [];
        return;
    end

    angles = zeros(numServos, 1);
    for i = 1:numServos
        cfg = configs(i);
        channel = channels(i) + 1; % MATLAB uses 1-based indexing
        pulse = pulses(channel);
        proportion = (pulse - cfg.min_pulse) / (cfg.max_pulse - cfg.min_pulse);
        angle = cfg.min_angle + proportion * (cfg.max_angle - cfg.min_angle);
        angles(i) = max(cfg.min_angle, min(cfg.max_angle, angle));
    end
end

function cfgs = servo_configs()
    cfgs = struct('min_angle', {}, 'max_angle', {}, 'min_pulse', {}, 'max_pulse', {});
    cfgs(1) = struct('min_angle', -pi/2, 'max_angle', pi/2, 'min_pulse', 500, 'max_pulse', 2500);
    cfgs(2) = struct('min_angle', -0.35, 'max_angle', 2.0, 'min_pulse', 500, 'max_pulse', 2500);
    cfgs(3) = struct('min_angle', -2.4, 'max_angle', 0.35, 'min_pulse', 500, 'max_pulse', 2500);
    cfgs(4) = struct('min_angle', -2.0, 'max_angle', 2.0, 'min_pulse', 500, 'max_pulse', 2500);
    cfgs(5) = struct('min_angle', -1.0, 'max_angle', 1.0, 'min_pulse', 800, 'max_pulse', 2200);
end

function channels = servo_channels()
    channels = [0, 1, 2, 3, 4];
end

function cmd = format_ssc32_command(pulses, travelTime)
    channels = 0:(numel(pulses) - 1);
    assignments = compose("#%dP%d", channels(:), pulses(:));
    duration = max(0, round(travelTime * 1000));
    if duration > 0
        cmd = strjoin([assignments; "T" + string(duration)], ' ');
    else
        cmd = strjoin(assignments, ' ');
    end
end

function close_serial(serialObj)
    if isempty(serialObj)
        return;
    end
    try
        if isvalid(serialObj)
            writeline(serialObj, ''); %#ok<TRYNC>
        end
    catch %#ok<CTCH>
    end
    clear serialObj
end
