
% function boxes = BoundPoles(pole_positions,width, height)
% % Returns a matrix where each row is a box around the poles.
% % boxes is [x_bottom_left, y_bottom_left, width, height]
%
% sz = size(pole_positions);
% num_poles = sz(1)/2;
% boxes = zeros(num_poles, 4);
% for i = 1:1:num_poles
%     boxes(i,:) = [pole_positions(i)-width/2, pole_positions(i+1)-height/2,...
%                   width, height];
% end
%
% end

function boxes = BoundPoles(pole_positions,width, height)
% Returns a matrix where each row is a box around the poles.
% boxes is [x_bottom_left, y_bottom_left, width, height]

sz = numel(pole_positions);

boxes = [];
for i = 1:2:sz
    i
    boxes = [boxes; pole_positions(i)-width/2, pole_positions(i+1)-height/2,...
        width, height];
end

end