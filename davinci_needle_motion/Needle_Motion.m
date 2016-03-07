%This is Russell's Test script that runs a needle rotation. 

function [needle] = Needle_Motion
close
clear
clc

%This plots the needle as it moves through space.


%% Loop Data
%steps:
N = 100;




%% Define the Needle Geometry and motion

n = 3;

switch n
    
    case 1
        
%Needle Type 1. (Ideal Curved Needle)
structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);

NeedleTransDes = struct('rot',[1 0 0 0]','trans', [0 0 0]');
Q = @(theta) [cos(-theta/2) sin(-theta/2)*[0 0 1]]';
pos = @(theta) -10*[cos(-theta+pi/2); sin(-theta+pi/2); 0];

    case 2
%Needle Type 2. Ideal Straight Needle
structNeedleGeometry = struct('straightL',10,'kinkAngle', 0, 'radius',0,'arc',0);

NeedleTransDes = struct('rot',[1 0 0 0]','trans', [0 0 0]');
Q = @(theta) [1 0*[0 0 1]]';
pos = @(theta) -.5*[theta; 0; 0];

    case 3
%Needle Type 3. non ideal curved Needle
structNeedleGeometry = struct('straightL',4,'kinkAngle', pi/7, 'radius',10,'arc',pi);

NeedleTransDes = struct('rot',[1 0 0 0]','trans', [0 0 0]');
Q = @(theta) [cos(-theta/2) sin(-theta/2)*[0 0 1]]';
pos = @(theta) -10*[cos(-theta+pi/2); sin(-theta+pi/2); 0];


end


theta = linspace(pi/2,1.5*pi,N);


%% Tissue Block.

switch n 


    case 2
        Tissue = [-20 -10 -10 -20 -20 -10 -10 -20; 5 5 -5 -5 5 5 -5 -5; 5 5 5 5 -5 -5 -5 -5];
        structTissueData =struct('center',[-15; 0; 0],'widthX',[10],'heightZ',[10],'depthY',[10]);
    otherwise
        Tissue = [-10  10  10 -10 -10  10  10 -10; -5 -5 -15 -15 -5 -5 -15 -15; 5 5 5 5 -5 -5 -5 -5];
        structTissueData =struct('center',[0; -10; 0],'widthX',[20],'heightZ',[10],'depthY',[10]);

    
end

%Tissue = [-20 -10 -10 -20 -20 -10 -10 -20; 5 5 -5 -5 5 5 -5 -5; 5 5 5 5 -5 -5 -5 -5];

%% Allocate the Needle force vectors
wrenchModeled = zeros(6,N);
wrenchFriction = zeros(6,N);
wrenchNorm = zeros(6,N);
wrenchCut = zeros(6,N);


%% Set up the force parameters
%older statement - Matparameters = [2 1 .2006  0 1.5]';
%after debugging -
Matparameters=struct('mus',0.16,'muk',0,'K',eye(3)*[0.0087; 2*0.0087; 4*0.0012;],'alpha',1.0622,'limit',1.6717);




%needleFig = figure('position',[500,500,1000,500]);
needleFig = figure;



%% Begin Loop
time = linspace(0,10,N);
for i = 1:N
    
        NeedleTransDes.rot  = Q(theta(i));
        NeedleTransDes.trans  = pos(theta(i));
    if( i == 1)
           [TissueStateOld  NeedlePts2Old]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,NeedleTransDes,100);
           wrenchNormOld = zeros(6,1);
           numCol = size(NeedlePts2Old,2);
           needleHead = NeedlePts2Old(:,numCol);
           needleTail = NeedlePts2Old(:,2);
           needleFrameOrigin = NeedlePts2Old(:,1);
           needle=struct('head',needleHead,'tail',needleTail,'frameOrigin',needleFrameOrigin);
    else
        TissueStateOld = TissueState;
        NeedlePts2Old  = NeedlePts2;
        wrenchNormOld  =  wrenchNorm(:,i-1);
    end
        

    [TissueState  NeedlePts2]=Needle_Tissue_Intersection(structTissueData,structNeedleGeometry,NeedleTransDes,100);
    

    
    
    %[wrenchModeled(:,i),wrenchFriction(:,i),wrenchNorm(:,i),wrenchCut(:,i)] = Needle_Cumulative_Forces_Step(TissueStateOld,NeedlePts2Old,TissueState,NeedlePts2,Matparameters,wrenchNormOld);

    

    
    
    clf
    axis([-20 20 -20 20]);
    axis square
    
    
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




%Plot the needle forces.
%forcesHand = figure;


% plot(time,wrenchNorm(1,:),time,wrenchNorm(2,:),time,wrenchNorm(3,:));
% legend('x','y','z');
% 
% torquesHand = figure;
% plot(time,wrenchNorm(4,:),time,wrenchNorm(5,:),time,wrenchNorm(6,:));
% legend('x','y','z');


end
