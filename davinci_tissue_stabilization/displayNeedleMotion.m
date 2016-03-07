function displayNeedleMotion(structTissueData,structNeedleGeometry,structNeedleData,N,flag,Tissue)
% This function is used to display needle motion
% Input: 
% 1. Transformation matrix from needle frame to tissue frame
% 2. Needle Geometry
% 3. Number of points on needle

%% calling Needle_Shape function to generate needle points
    [TissueState, NeedlePts2] = Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,structNeedleData,N,flag)

%%
    clf;
    axis([-20 20 -20 20]);
    axis square;
    
    
    %Plot the box.
    line(Tissue(1,:),Tissue(2,:),'color','blue');
    
    
    
    %Plot the Needle
    outside = find(TissueState ==0);
    if(~isempty(outside))
        for k = outside'
            line(NeedlePts2(1,k:k+1),NeedlePts2(2,k:k+1),'marker','none','color','green');
        end
    end
    
    inside  = find(TissueState ==1);
    if(~isempty(inside))
        for k = inside'
            line(NeedlePts2(1,k:k+1),NeedlePts2(2,k:k+1),'marker','none','color','red');
        end
    end
    
    
    neither  = find(TissueState > 0 & TissueState < 1);
    if(~isempty(neither))
        for k = neither'
            line(NeedlePts2(1,k:k+1),NeedlePts2(2,k:k+1),'marker','none','color','yellow');
        end
    end

    %Plot Needle.
    
    pause(.01);
end


