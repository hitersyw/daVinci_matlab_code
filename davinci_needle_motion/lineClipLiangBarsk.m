function [p1 p2] = lineClipLiangBarsk(winDims,p1,p2)

%check sizes
if size(winDims,1) == 2 && size(winDims,2) == 3
    winDims=winDims';
elseif size(winDims,1) ~= 3 || size(winDims,2) ~= 2
    error('invalid size of parameter winDims - must be 3x2');
end
if size(p1,1) == 1 && size(p1,2) == 3
    p1=p1';
elseif size(p1,1) ~= 3 || size(p1,2) ~= 1
    error('invalid size of parameter p1 - must be 3x1');
end
if size(p2,1) == 1 && size(p2,2) == 3
    p2=p2';
elseif size(p2,1) ~= 3 || size(p1,2) ~= 1
    error('invalid size of parameter p2 - must be 3x1');
end

xmin = winDims(1,1);
xmax = winDims(1,2);
ymin = winDims(2,1);
ymax = winDims(2,2);
zmin = winDims(3,1);
zmax = winDims(3,2);

p1x = p1(1);
p1y = p1(2);
p1z = p1(3);

p2x = p2(1);
p2y = p2(2);
p2z = p2(3);

if xmin >xmax
    error('xmin > xmax');
end
if ymin > ymax
    error('ymin > ymax');
end
if zmin > zmax
    error('zmin > zmax');
end

%check if both points are outside the window

if ((p1(1) > xmax || p1(1) < xmin) || (p1(2) > ymax || p1(2) < ymin) || (p1(3) > zmax || p1(3) < zmin)) &&...
        ((p2(1) > xmax || p2(1) < xmin) || (p2(2) > ymax || p2(2) < ymin) || (p2(3) > zmax || p2(3) < zmin))
    p1 = [0 0 0]';
    p2 = [0 0 0]';
    
else
    
    u1=0;
    u2=1;
    dx = p2x-p1x;
    dy = p2y-p1y;
    dz = p2z-p1z;
    
    
    [clip, u1, u2] = clipTest(-dx, p1x-xmin, u1, u2);
    if clip
        [clip, u1, u2] = clipTest(dx, xmax-p1x, u1, u2);
        if clip
            [clip, u1, u2] = clipTest(-dy, p1y-ymin, u1, u2);
            if clip
                [clip, u1, u2] = clipTest(dy, ymax-p1y, u1, u2);
                if clip
                    [clip, u1, u2] = clipTest(-dz, p1z-zmin, u1, u2);
                    if clip
                        [clip, u1, u2] = clipTest(dz, zmax-p1z, u1, u2);
                        if clip
                            if u2 < 1
                                p2 = [p1x + u2*dx, p1y + u2*dy, p1z + u2*dz]';
                            end
                            if u1 > 0
                                p1 = [p1x + u1*dx, p1y + u1*dy, p1z + u1*dz]';
                            end
                        end
                    end
                end
            end
        end
    end
end