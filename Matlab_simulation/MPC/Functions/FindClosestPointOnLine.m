function closestpoint = FindClosestPointOnLine(position, line_start, line_end)
%NOT WORKING... DUNNO WHY....
% diff = line_end - line_start;
% norm2 = norm(diff,2);
% 
% if norm2 == 0
%     closestpoint = line_start;
% else
%     u = ((position(1)-line_start(1))*diff(1) + (position(2)-line_start(2))*diff(2))/norm2;
%     if u <= 0
%         closestpoint = line_start;
%     elseif u >= 1
%         closestpoint = line_end;
%     else
%         closestpoint = line_start + u*diff;
%     end
% end

%but this works thanks to stackoverflow
    s2p = position - line_start;
    s2e = line_end - line_start;
    sq_s2e = s2e(1)^2+s2e(2)^2;
    dot = s2p(1)*s2e(1)+s2p(2)*s2e(2);
    t = max(0, min(1, dot/sq_s2e));
    %if t <= 0
    %    closestpoint = line_start;
    %elseif t >= 1
    %    closestpoint = line_end;
    %else
        closestpoint = [line_start(1)+s2e(1)*t, line_start(2)+s2e(2)*t];
    %end
end


