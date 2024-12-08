%% Allocation json files
% sequence_of_input_commands [roll,pitch,yaw,throttle,forward,roll_ff, pitch_ff, yaw_ff]

% VTOL:
alo_mat_fb=allocation_matrix_normalized;% shorter name for readability
TriCopterKU_vtol_manual_control_gain_positive=[
        [ 0.0           , alo_mat_fb(1,2)   ,0      ,alo_mat_fb(1,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(2,1), alo_mat_fb(2,2)   ,0      ,alo_mat_fb(2,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(3,1), alo_mat_fb(3,2)   ,0      ,alo_mat_fb(3,4),0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,s4_alloc_pos_gain,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s6_alloc_pos_gain ,0.0            ,s6_alloc_pos_gain_fwd_cmd ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s7_alloc_pos_gain ,0.0            ,-s7_alloc_neg_gain_fwd_cmd,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,s8_alloc_gain_pos,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,s13_alloc_gain_neg ,-s13_alloc_gain_neg,s13_alloc_gain_pos ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,s14_alloc_gain_neg ,s14_alloc_gain_pos ,s14_alloc_gain_pos ,0.0]];

TriCopterKU_vtol_manual_control_gain_negative=[
        [ 0.0           , alo_mat_fb(1,2)   ,0      ,alo_mat_fb(1,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(2,1), alo_mat_fb(2,2)   ,0      ,alo_mat_fb(2,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(3,1), alo_mat_fb(3,2)   ,0      ,alo_mat_fb(3,4),0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,s4_alloc_neg_gain,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s6_alloc_neg_gain ,0.0            , s6_alloc_neg_gain_fwd_cmd,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s7_alloc_neg_gain ,0.0            ,-s7_alloc_pos_gain_fwd_cmd,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,s8_alloc_gain_neg,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0 ,-s13_alloc_gain_pos,s13_alloc_gain_neg,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0 ,s14_alloc_gain_neg ,s14_alloc_gain_neg ,0.0]];
