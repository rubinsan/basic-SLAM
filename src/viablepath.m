function viable = viablepath (pA, pB, obstacles)

% This function evaluates the feasibility of a path between two points: A and B
% within a map containing obstacles (obstacles)
% It receives the (x, y) coordinates of two points (A, B) and a
% vector of obstacles (2 x number of obstacles).
% Returns a confirmation of the path's feasibility.

% y=m*x+b

m=(pB(2)-pA(2))/(pB(1)-pA(1)); % infinite result bug 
b=pA(2)-m*pA(1);  
vectorAB=pB-pA;
rows = size(obstacles,1);
viable=1;
for i = 1:(rows)  
    inside=0;
    obstaclepoint=obstacles(i,:);
    yC=m*obstaclepoint(1)+b;
    if ((yC<=obstaclepoint(2)+0.5) && (yC>=obstaclepoint(2)-0.5))  % y range check
        inside=1;
    end
    xC=(obstaclepoint(2)-b)/m;
    if ((xC<=obstaclepoint(1)+0.5) && (xC>=obstaclepoint(1)-0.5))  % x range check
        inside=1;
    end
    if (m==inf || m==-inf)
        if (pA(1)==obstaclepoint(1) || pB(1)==obstaclepoint(1))
            long=min(pA(2),pB(2)): 0.1 :max(pA(2),pB(2));
            inside=find(long==obstaclepoint(2));
            if inside~=0
                viable=0;
                break;
            end
        end
    end    
    if (inside==1) % if belongs to the line
        vectorAC=[(obstaclepoint(1)-pA(1)),(obstaclepoint(2)-pA(2))];
        dotAC=dot(vectorAB, vectorAC);
        dotAB=dot(vectorAB, vectorAB);
        if (dotAC > 0 && dotAC < dotAB)
            viable=0;
            break
        end
    end 
end
end
