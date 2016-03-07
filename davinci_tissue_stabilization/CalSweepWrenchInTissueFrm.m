%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    This function is used to calculate sweeping wrench in tissue frame   %
%                                                                         %
%    needle frame == body frame                                           %
%    tissue frame == spacial frame                                        %
%                                                                         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% main function

function [sweep_force_vec] = CalSweepWrenchInTissueFrm
    close;
    clear;
    clc;
    %% loop date
    N = 150; % steps
    
    %% Define the Needle Geometry and motion
    %Needle Type 3. non ideal curved Needle
    structNeedleGeometry = struct('straightL',0,'kinkAngle', 0, 'radius',10,'arc',pi);
    %% Define hybrid velocity
    hyVelocity = struct('transV',[0 0 0]','angV',[0 0 -0.2*pi]');
    %% initial transformation matrix (zero configuration)
    needle_tip_pos_init = [30;0;0]; % the initial need tip position wrt spatial frame
%     transMatFrmNeedleToTissue = struct('rot',[-1 0 0;0 1 0;0 0 -1],'trans',...
%                 needle_tip_pos_init - [-1 0 0;0 1 0;0 0 -1] * structNeedleGeometry.radius * [-1 0 0]');
%             
    transMatFrmNeedleToTissue.rot = [-1 0 0;0 1 0;0 0 -1];
    transMatFrmNeedleToTissue.trans = needle_tip_pos_init - transMatFrmNeedleToTissue.rot * structNeedleGeometry.radius * [-1 0 0]';
    %% Define tissue parameters
    point_q = [1 -5 1]'; % a constant point on tissue surface defined in tissue frame
    normal_vec = [0 1 0]'; % normal vector of tissue surface defined in tissue frame
    K = [0.0138 0 0;0 0.0188 0;0 0 0]; % tissue stiffness matrix
    tissue = [-15  15  15 -15 -15  15  15 -15; -5 -5 -15 -15 -5 -5 -15 -15];
    structTissueData =struct('center',[0; -10; 0],'widthX',[30],'heightZ',[10],'depthY',[10]);
    %% begin loop
    time = linspace(0,15.1,N); % integral time interval
    sweep_force_init = [0.0 0.0 0.0]; % inital sweeping force
    sweep_force_vec = sweep_force_init;
    sweep_force_current_time = sweep_force_init'; % sweeping force at time zero
    
    wft_initial = [0 0 0]; %initial incremental sweeping force
    wtf_vec = wft_initial;
    
    sweep_torque_init = [0.0 0.0 0.0]; % inital sweeping force
    sweep_torque_vec = sweep_torque_init;
    sweep_torque_current_time = sweep_torque_init'; % sweeping force at time zero
    
    wmt_initial = [0 0 0]; %initial incremental sweeping force
    wmt_vec = wmt_initial;
    for i = 1:N - 1
        current_time_step = time(i);
        
        if( i == 1 )
            integral_limit = FindTheta( transMatFrmNeedleToTissue, point_q, normal_vec, structNeedleGeometry);
            if (isnan(integral_limit.max) || isnan(integral_limit.min))
                previous_trans_matrix = transMatFrmNeedleToTissue;
                
                sweep_force_vec = [sweep_force_vec; zeros(1,3)];
                wtf_vec = [wtf_vec; zeros(1,3)];
                
                sweep_torque_vec = [sweep_torque_vec; zeros(1,3)];
                wmt_vec = [wmt_vec; zeros(1,3)];
%                 continue;
            else
                % remember the initial transformation matrix in first step
                % so that it can be used to calculate next time step trans-
                % formation matrix
                previous_trans_matrix = transMatFrmNeedleToTissue;
            end
        elseif (current_time_step <= 10)
            previous_time_step = time(i-1);
            current_time_step = time(i);
            next_time_step = time(i+1);
            
            [current_trans_matrix, spVelocity] = CalTransMatrix(previous_trans_matrix,previous_time_step,current_time_step,hyVelocity);
            transMatFrmNeedleToTissue = current_trans_matrix;
            
            % calculate the position of needle tip and tail w/rt tissue frm
            needle_tip_pos = transMatFrmNeedleToTissue.trans + transMatFrmNeedleToTissue.rot * structNeedleGeometry.radius * [-1 0 0]';
            needle_tail_pos = transMatFrmNeedleToTissue.trans + transMatFrmNeedleToTissue.rot * structNeedleGeometry.radius * [1 0 0]';
%             dist_tip_tail = sqrt((needle_tip_pos(1) - needle_tail_pos(1))^2+ (needle_tip_pos(2) - needle_tail_pos(2))^2);
            
%             disp(transMatFrmNeedleToTissue.trans);
%             disp(dist_tip_tail);
%             disp(hyVelocity.transV);
            integral_limit = FindTheta(transMatFrmNeedleToTissue, point_q, normal_vec, structNeedleGeometry);
            
            if (isnan(integral_limit.max) || isnan(integral_limit.min))
      
                sweep_force_vec = [sweep_force_vec; zeros(1,3)];
                wtf_vec = [wtf_vec; zeros(1,3)];
                
                sweep_torque_vec = [sweep_torque_vec; zeros(1,3)];
                wmt_vec = [wmt_vec; zeros(1,3)];
                
                previous_trans_matrix = current_trans_matrix;
                
                
%                 continue;
            else
                previous_trans_matrix = current_trans_matrix;
                boolean = checkHybridVelo(hyVelocity, needle_tip_pos, needle_tail_pos, normal_vec);
                if (~boolean)
                % checking if need to change translation of hybrid velocity
                    hyVelocity.transV = [0;-1;0];
                    hyVelocity.angV = [0;0;0];
                end
                [sweep_force_next_time, wtf] = calSweepForceInTissueFrm(structNeedleGeometry,...
                    K, transMatFrmNeedleToTissue, spVelocity, integral_limit, sweep_force_current_time, current_time_step, next_time_step);
                
                [sweep_torque_next_time, wmt] = calSweepTorqueInTissueFrm(structNeedleGeometry,...
                    K, transMatFrmNeedleToTissue, spVelocity, integral_limit, sweep_torque_current_time, current_time_step, next_time_step);
                % appending to sweeping force vector
                sweep_force_vec = [sweep_force_vec; sweep_force_next_time'];
                
                % appending to incremental sweeping force vector
                wtf_vec = [wtf_vec; wtf'];
                
                
                sweep_torque_vec = [sweep_torque_vec; sweep_torque_next_time'];
                wmt_vec = [wmt_vec; wmt'];
                
                sweep_force_current_time = sweep_force_next_time;
                sweep_torque_current_time = sweep_torque_next_time;
            end
        else
            previous_time_step = time(i-1);
            current_time_step = time(i);
            next_time_step = time(i+1);
            
            [current_trans_matrix, spVelocity] = CalTransMatrix(previous_trans_matrix,previous_time_step,current_time_step,hyVelocity);
            transMatFrmNeedleToTissue = current_trans_matrix;
            
            % calculate the position of needle tip and tail w/rt tissue frm
            needle_tip_pos = transMatFrmNeedleToTissue.trans + transMatFrmNeedleToTissue.rot * structNeedleGeometry.radius * [-1 0 0]';
            needle_tail_pos = transMatFrmNeedleToTissue.trans + transMatFrmNeedleToTissue.rot * structNeedleGeometry.radius * [1 0 0]';
%             dist_tip_tail = sqrt((needle_tip_pos(1) - needle_tail_pos(1))^2+ (needle_tip_pos(2) - needle_tail_pos(2))^2);
            
%             disp(transMatFrmNeedleToTissue.trans);
%             disp(dist_tip_tail);
%             disp(hyVelocity.transV);
            integral_limit = FindTheta(transMatFrmNeedleToTissue, point_q, normal_vec, structNeedleGeometry);
            
            if (isnan(integral_limit.max) || isnan(integral_limit.min))
      
                sweep_force_vec = [sweep_force_vec; zeros(1,3)];
                wtf_vec = [wtf_vec; zeros(1,3)];
                
                sweep_torque_vec = [sweep_torque_vec; zeros(1,3)];
                wmt_vec = [wmt_vec; zeros(1,3)];
                
                previous_trans_matrix = current_trans_matrix;
%                 continue;
            else
                previous_trans_matrix = current_trans_matrix;
   
                hyVelocity.transV = [0;0;0];
                hyVelocity.angV = [0;0;-0.2*pi];
                [sweep_force_next_time, wtf] = calSweepForceInTissueFrm(structNeedleGeometry,...
                    K, transMatFrmNeedleToTissue, spVelocity, integral_limit, sweep_force_current_time, current_time_step, next_time_step);
                
                [sweep_torque_next_time, wmt] = calSweepTorqueInTissueFrm(structNeedleGeometry,...
                    K, transMatFrmNeedleToTissue, spVelocity, integral_limit, sweep_torque_current_time, current_time_step, next_time_step);
                % appending to sweeping force vector
                sweep_force_vec = [sweep_force_vec; sweep_force_next_time'];
                
                % appending to incremental sweeping force vector
                wtf_vec = [wtf_vec; wtf'];
                
                sweep_torque_vec = [sweep_torque_vec; sweep_torque_next_time'];
                wmt_vec = [wmt_vec; wmt'];
                
                sweep_force_current_time = sweep_force_next_time;
                sweep_torque_current_time = sweep_torque_next_time;
            end
        end
        displayNeedleMotion(structTissueData,structNeedleGeometry,transMatFrmNeedleToTissue,100,true,tissue);
    end
    sweep_force_table = array2table(sweep_force_vec, 'VariableNames',{'x' 'y' 'z'});
    
    wtf_table = array2table(wtf_vec, 'VariableNames',{'x' 'y' 'z'});
    
    sweep_torque_table = array2table(sweep_torque_vec, 'VariableNames',{'x' 'y' 'z'});
    
    wmt_table = array2table(wmt_vec, 'VariableNames',{'x' 'y' 'z'});
    
    figure % new figure
    ax1 = subplot(2,2,1);
    ax2 = subplot(2,2,2);
    
    ax3 = subplot(2,2,3);
    ax4 = subplot(2,2,4);

    plot(ax1,time,wtf_table.x,'--',time,wtf_table.y,'-');
    title(ax1,'increment sweeping force');
    
    plot(ax2,time,sweep_force_table.x,'--',time,sweep_force_table.y,'-');
    title(ax2,'sweeping force');
    
    plot(ax3,time,wmt_table.x,'--',time,wmt_table.y,'-',time,wmt_table.z,'.-');
    title(ax3,'increment sweeping torque');
    
    plot(ax4,time,sweep_torque_table.x,'-',time,sweep_torque_table.y,'--',time,sweep_torque_table.z,'.-');
    title(ax4,'sweeping torque');
    
end

%% subfunction to check the need to change hybrid velocity
% checking if need to change translation of hybrid velocity
function [boolean] = checkHybridVelo(hyVelocity, needle_tip_pos, needle_tail_pos, normal_vec)
    boolean = 1;
    if (isequal(hyVelocity.transV,[0;-1;0]) && isequal(hyVelocity.angV,[0;0;0]))
    % check if translation velocity == [0;0;0], 
    % YES: jump to next iteration
        return;
    else
    % NO: change translation velocity to [0;0;0]
        angle = calAngleBtwTwoVectors(needle_tip_pos, needle_tail_pos, normal_vec);
        if (angle <= pi/2)
        % This is the scenario that when need tip starts to bite
        % tissue
            boolean = 0;
        end
    end
end

%% subfunction the angle between two vectors
function angle = calAngleBtwTwoVectors(point_1, point_2, vector)
    a = point_1 - point_2; % a is a vector pointing from point_2 to point_1
    b = vector;
    angle = atan2(norm(cross(a,b)),dot(a,b)); % the result is in rad unit
end