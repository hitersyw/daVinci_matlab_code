function [TissueState NeedlePtList] = Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,structNeedleData,N,flag)
%#codegen
%TissueState: 1 if needle segment is in tissue
%             0 if not
%             0-1 represents proportion of needle segment in tissue
%            -1-0 represent portions of the needle exiting the tissue
%This code implicitly assumes that the needle is inside one block of tissue
%Violating this assumption may break the code.
%and that it will not stick in a block, come out, and go back in.

%NeedlePtList: coordinates in Cartesian space of the N points defining the N+1 needle segments


%Goal: find the locations where the needle actually enters and exits the tissue.
%This problem is performed in 3 space.
%This is the numerical version for N needle segments.

%structNeedleData =     struct('trans',[0; 0; 0],'rot',[a;b;c;d];);
%structNeedleGeometry = struct('straightL',4.5,'kinkAngle', -15/180*pi, 'radius',11.5,'arc',150*pi/180);
%structTissueData =     struct('center',[0; 0; 0],'widthX',[0],'heightZ',[0],'depthY',[0]);




%Generate the point list.
NeedlePtList = Needle_Shape(structNeedleData,structNeedleGeometry,N,flag);


%TissueState = -1*ones(N,1);


%perform the clipping algorithm.
xMin = structTissueData.center(1)-structTissueData.widthX/2;
xMax = structTissueData.center(1)+structTissueData.widthX/2;
yMin = structTissueData.center(2)-structTissueData.depthY/2;
yMax = structTissueData.center(2)+structTissueData.depthY/2;
zMin = structTissueData.center(3)-structTissueData.heightZ/2;
zMax = structTissueData.center(3)+structTissueData.heightZ/2;


winDims = [xMin xMax;...
           yMin yMax;...
           zMin zMax;];
       
       
%Check to see if any needle Points are fully inside the tissue.

testVals = (NeedlePtList(1:3,:) >= repmat(winDims(:,1),1,N+1)) & (NeedlePtList(1:3,:) <= repmat(winDims(:,2),1,N+1));

testSums = sum(testVals,1);

insidePts = find(testSums == 3);

%Get the indices:
if(~isempty(insidePts))
       
TissueState = zeros(N,1);



TissueState(insidePts(1:end-1)) = ones(length(insidePts)-1,1);
if(length(insidePts) > 1)
if(insidePts(1) ==1 && insidePts(2) > 2)
   
    TissueState(1) = -eps;
    
end

if(insidePts(end) == N+1 && insidePts(end-1) < N)
   
    TissueState(end) = eps;
    
end
end
%The base of the needle is not in the tissue!!
if(insidePts(1) > 1)
    
    p1 = NeedlePtList(1:3,insidePts(1)-1);
    p2 = NeedlePtList(1:3,insidePts(1));
    [p1Out p2Out] = lineClipLiangBarsk(winDims,p1,p2);
    
    d1 = norm(p1Out-p1);
    d2 = norm(p2Out-p2);
    
    if(d1 > d2)
        lambda = -(1-norm(p1Out-p2Out)/norm(p1-p2));
        %lambda = -(norm(p1Out-p2Out)/norm(p1-p2));
    
    else
        lambda = -(norm(p1Out-p2Out)/norm(p1-p2));
    end
    
    TissueState(insidePts(1)-1) = lambda;
    
end


%The Tip of the needle is not in the tissue
if(insidePts(end) < N+1)
    p1 = NeedlePtList(1:3,insidePts(end));
    p2 = NeedlePtList(1:3,insidePts(end)+1);
    
    
    [p1Out p2Out] = lineClipLiangBarsk(winDims,p1,p2);
    
    d1 = norm(p1Out-p1);
    d2 = norm(p2Out-p2);
    
    
    if(d2 > d1)
        lambda = norm(p1Out-p2Out)/norm(p1-p2);
    else
        lambda = (1-norm(p1Out-p2Out)/norm(p1-p2));
    end
    
    
    TissueState(insidePts(end)) = lambda;

end


%%% debug


% figure
% hold on; plot3(p1(1),p1(2),p1(3),'o')
% hold on; plot3(p2(1),p2(2),p2(3),'ro')
% wX = structTissueData.widthX;
% hZ = structTissueData.heightZ;
% dY = structTissueData.depthY;
% c  = structTissueData.center;
% Tissue = repmat(c,1,12)+[-wX/2  wX/2  wX/2 -wX/2 -wX/2  wX/2  wX/2 -wX/2 -wX/2  wX/2  wX/2 -wX/2;...
%     dY/2  dY/2 -dY/2 -dY/2  dY/2  dY/2  dY/2  dY/2 -dY/2 -dY/2  dY/2  dY/2;...
%     hZ/2  hZ/2  hZ/2  hZ/2  hZ/2  hZ/2 -hZ/2 -hZ/2 -hZ/2 -hZ/2 -hZ/2 -hZ/2];
% line(Tissue(1,:),Tissue(2,:),Tissue(3,:))
% hold on; plot3(p2Out(1),p2Out(2),p2Out(3),'rx')
% hold on; plot3(p1Out(1),p1Out(2),p1Out(3),'x')

%%%

    
else
    TissueState = zeros(N,1);
end