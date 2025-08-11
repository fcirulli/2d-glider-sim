function [value ,isterminal,direction] = GroundHit(~,y)
  % Event function stops integration when height = 0

  value = y(4); % glider crosses y = 0
  isterminal = 1; % Stop integration
  direction = -1; % Only triggered if glider is falling
end
