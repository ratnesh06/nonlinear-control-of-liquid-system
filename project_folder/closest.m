% % computes the closest discretized state to snext
function [r, s] = closest(snext)
% Computes the closest discretized state to snext

global h1 h2 delta;

% Handle edge cases for Tank 1
if snext(1) > h1(end) + delta
    r = length(h1); % Cap at the maximum index
elseif snext(1) < h1(1) - delta
    r = 1; % Minimum index
else
    % Find the closest index
    r = find(abs(h1 - snext(1)) <= delta, 1);
end

% Handle edge cases for Tank 2
if snext(2) > h2(end) + delta
    s = length(h2); % Cap at the maximum index
elseif snext(2) < h2(1) - delta
    s = 1; % Minimum index
else
    % Find the closest index
    s = find(abs(h2 - snext(2)) <= delta, 1);
end

% Default to the closest point if no valid match is found
if isempty(r), r = length(h1); end
if isempty(s), s = length(h2); end

end
