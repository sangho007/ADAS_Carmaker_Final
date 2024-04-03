
function  localPoints_map = Global2Local_path(path, start_x, start_y,yaw)

persistent yaw_prev

if isempty(yaw_prev)
    yaw_prev = yaw;
end

Yaw_ego =yaw;

% Create a Global2Local_class object for converting the entire map
g2l_map = Global2Local_class(length(path));

% Convert the entire map to local coordinates
localPoints_map = g2l_map.convert(path, Yaw_ego, start_x, start_y).LocalPoints;

end