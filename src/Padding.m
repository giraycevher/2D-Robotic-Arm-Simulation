  %% This Code essantial for Double S-Cruve Trajectory
  %% Theta2 or theta1 possibly may not same length of time vector
  %% Padding code fixing that diffrent time vector size

    if abs(t_seg(end) - T_max) > dt/2
        t_seg = t_seg(1:end-1);
        numstep = length(t_seg);
    end

    if Ttotal_1 < T_max
        padding = numstep-length(q1);
        
        
        q_1_final = q1(end);
        q1 = [q1;ones(padding,1)*q_1_final];
        a1 = [a1;zeros(padding,1)];
        v1 = [v1;zeros(padding,1)];
    end
    if  Ttotal_2 < T_max
        padding = numstep-length(q2);r
        q_2_final = q2(end);
    
        q2 = [q2;ones(padding,1)*q_2_final];
        a2 = [a2;zeros(padding,1)];
        v2 = [v2;zeros(padding,1)];
    end
    
   
    min_len_q = min(length(q1), length(q2));
    if length(q1) ~= length(q2)
        q1 = q1(1:min_len_q);
        q2 = q2(1:min_len_q);
        v1 = v1(1:min_len_q);
        v2 = v2(1:min_len_q);
        a1 = a1(1:min_len_q);
        a2 = a2(1:min_len_q);
        t_seg = t_seg(1:min_len_q);
        numstep = min_len_q;
    end
%%
