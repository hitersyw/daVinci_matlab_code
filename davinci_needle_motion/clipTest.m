function [clip,u1,u2] = clipTest(p,q,u1,u2)

clip = 1;

if p < 0 %line goes from outside plane to inside plane
    r=q/p;
    if r > u2
        clip = 0;
    elseif r > u1
        u1=r;
    end
    
else
    if p > 0 %line goes from inside plane to outside plane
        r = q/p;
        if r < u1
            clip = 0;
        elseif r < u2
            u2=r;
        end
    else
        %thus p=0 and the line is parallel to the clipping boundary
        if q<0
            %line is outside clipping boundary
            clip = 0;
        end
    end
end