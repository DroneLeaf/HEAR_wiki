%% Type json file
minimum_actuator_cmds=[0.0,0.0,0.0,s4_alloc_min,0.0,s6_alloc_min,s7_alloc_min,s8_alloc_min,s13_alloc_min,s14_alloc_min];
maximum_actuator_cmds=[1.0,1.0,1.0,s4_alloc_max,0.0,s6_alloc_max,s7_alloc_max,s8_alloc_max,s13_alloc_max,s14_alloc_max];

minimum_actuator_cmds_plane=[0.0,0.0,0.0,s4_alloc_min_plane,0.0,s6_alloc_min_plane,s7_alloc_min_plane,s8_alloc_min,s13_alloc_min,s14_alloc_min];
maximum_actuator_cmds_plane=[0.0,0.0,0.0,s4_alloc_max_plane,0.0,s6_alloc_max_plane,s7_alloc_max_plane,s8_alloc_max,s13_alloc_max,s14_alloc_max];

idle_io_cmds=[0.0,0.0,0.0,s4_io_idle ,s4_io_idle   ,s6_io_idle ,s7_io_idle,s8_idle_io,s13_idle_io   ,s14_idle_io   ,0.0   ,0.0];

disarmed_io_cmds=[0.0,0.0,0.0,s4_io_idle   ,s4_io_idle  ,s6_io_idle ,s7_io_idle,s8_idle_io,s13_idle_io   ,s14_idle_io,0.0   ,0.0];

minimum_io_range=[0.0,0.0,0.0,0.0,0.0,0.0   ,s7_io_range_min , 0.0   ,0.0,0.0,0.0,0.0];
maximum_io_range=[1.0,1.0,1.0,1.0,0.0,s6_io_range_max,1.0   , 1.0   ,1.0,1.0,0.0,0.0];

minimum_io_range_plane=[0.0,0.0,0.0,0.0,0.0,0.0   ,0.0   ,0.0    ,0.0,0.0,0.0,0.0];
maximum_io_range_plane=[1.0,0.0,0.0,1.0,0.0,1.0   ,1.0   , 1.0   ,1.0,0.0,0.0,1.0];

minimum_angle_deg_vtol=[0.0,0.0,0.0,s4_angle_at_min_pwm,s4_angle_at_min_pwm,s6_angle_at_min_pwm,s7_angle_at_min_pwm,s8_angle_at_min_pwm     ,0.0,0.0,0.0,0.0];
maximum_angle_deg_vtol=[0.0,0.0,0.0,s4_angle_at_max_pwm,s4_angle_at_max_pwm,s6_angle_at_max_pwm,s7_angle_at_max_pwm,s8_angle_at_max_pwm_vtol,0.0,0.0,0.0,0.0];

% %% Allocation json files
% % sequence_of_input_commands [roll,pitch,yaw,throttle,forward,roll_ff, pitch_ff, yaw_ff]
% 
% % VTOL:
% alo_mat_fb=allocation_matrix_normalized;% shorter name for readability
% TriCopterKU_vtol_manual_control_gain_positive=[
%         [ 0.0           , alo_mat_fb(1,2)   ,0      ,alo_mat_fb(1,4),0.0    ,0.0    ,0.0    ,0.0];
%         [alo_mat_fb(2,1), alo_mat_fb(2,2)   ,0      ,alo_mat_fb(2,4),0.0    ,0.0    ,0.0    ,0.0];
%         [alo_mat_fb(3,1), alo_mat_fb(3,2)   ,0      ,alo_mat_fb(3,4),0.0    ,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,s4_alloc_pos_gain,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,s6_alloc_pos_gain ,0.0            ,s6_alloc_pos_gain_fwd_cmd ,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,s7_alloc_pos_gain ,0.0            ,-s7_alloc_neg_gain_fwd_cmd,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,s8_alloc_gain_pos,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,s13_alloc_gain_neg ,-s13_alloc_gain_neg,s13_alloc_gain_pos ,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,s14_alloc_gain_neg ,s14_alloc_gain_pos ,s14_alloc_gain_pos ,0.0]];
% 
% TriCopterKU_vtol_manual_control_gain_negative=[
%         [ 0.0           , alo_mat_fb(1,2)   ,0      ,alo_mat_fb(1,4),0.0    ,0.0    ,0.0    ,0.0];
%         [alo_mat_fb(2,1), alo_mat_fb(2,2)   ,0      ,alo_mat_fb(2,4),0.0    ,0.0    ,0.0    ,0.0];
%         [alo_mat_fb(3,1), alo_mat_fb(3,2)   ,0      ,alo_mat_fb(3,4),0.0    ,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,s4_alloc_neg_gain,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,s6_alloc_neg_gain ,0.0            , s6_alloc_neg_gain_fwd_cmd,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,s7_alloc_neg_gain ,0.0            ,-s7_alloc_pos_gain_fwd_cmd,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,s8_alloc_gain_neg,0.0    ,0.0    ,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0 ,-s13_alloc_gain_pos,s13_alloc_gain_neg,0.0];
%         [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0 ,s14_alloc_gain_neg ,s14_alloc_gain_neg ,0.0]];


%% Allocation json files
% This allocation fixes canard, elevator no fwd stick

% sequence_of_input_commands [roll,pitch,yaw,throttle,forward,roll_ff, pitch_ff, yaw_ff]

% VTOL:
alo_mat_fb=allocation_matrix_normalized;% shorter name for readability
TriCopterKU_vtol_manual_control_gain_positive=[
        [ 0.0           , alo_mat_fb(1,2)   ,0      ,alo_mat_fb(1,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(2,1), alo_mat_fb(2,2)   ,0      ,alo_mat_fb(2,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(3,1), alo_mat_fb(3,2)   ,0      ,alo_mat_fb(3,4),0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s6_alloc_pos_gain ,0.0            ,s6_alloc_pos_gain_fwd_cmd ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s7_alloc_pos_gain ,0.0            ,-s7_alloc_neg_gain_fwd_cmd,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,s8_alloc_gain_pos,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                ,-s13_alloc_gain_neg,s13_alloc_gain_pos ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                ,s14_alloc_gain_pos ,s14_alloc_gain_pos ,0.0]];

TriCopterKU_vtol_manual_control_gain_negative=[
        [ 0.0           , alo_mat_fb(1,2)   ,0      ,alo_mat_fb(1,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(2,1), alo_mat_fb(2,2)   ,0      ,alo_mat_fb(2,4),0.0    ,0.0    ,0.0    ,0.0];
        [alo_mat_fb(3,1), alo_mat_fb(3,2)   ,0      ,alo_mat_fb(3,4),0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s6_alloc_neg_gain ,0.0            , s6_alloc_neg_gain_fwd_cmd,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,s7_alloc_neg_gain ,0.0            ,-s7_alloc_pos_gain_fwd_cmd,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,s8_alloc_gain_neg,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0 ,-s13_alloc_gain_pos,s13_alloc_gain_neg,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0 ,s14_alloc_gain_neg ,s14_alloc_gain_neg ,0.0]];




% Plane:
TriCopterKU_vtol_manual_control_plane_gain_positive=TriCopterKU_vtol_manual_control_gain_positive;
TriCopterKU_vtol_manual_control_plane_gain_negative=TriCopterKU_vtol_manual_control_gain_negative;
%% Post Allocation Motor Bias json file
% TriCopterKU_vtol_manual_control=[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  -s13_alloc_gain_neg,-s14_alloc_gain_neg];
% TriCopterKU_vtol_manual_control_plane=[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0];

%% Post Allocation Motor Bias json file
% elevator disengaged from fwd cmd
TriCopterKU_vtol_manual_control=[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0,0.0];
TriCopterKU_vtol_manual_control_plane=[ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0];

%% Write to json - UAV Type Json
% Calculated
write_value_to_JSON_file("ToConfigurations/general.json","minimum_actuator_cmds",minimum_actuator_cmds);
write_value_to_JSON_file("ToConfigurations/general.json","maximum_actuator_cmds",maximum_actuator_cmds);

write_value_to_JSON_file("ToConfigurations/general.json","idle_io_cmds",idle_io_cmds);
write_value_to_JSON_file("ToConfigurations/general.json","disarmed_io_cmds",disarmed_io_cmds);

write_value_to_JSON_file("ToConfigurations/general.json","minimum_io_range",minimum_io_range);
write_value_to_JSON_file("ToConfigurations/general.json","maximum_io_range",maximum_io_range);

write_value_to_JSON_file("ToConfigurations/general.json","minimum_angle_deg_vtol",minimum_angle_deg_vtol);
write_value_to_JSON_file("ToConfigurations/general.json","maximum_angle_deg_vtol",maximum_angle_deg_vtol);

% Plane related - needs revision
write_value_to_JSON_file("ToConfigurations/general.json","minimum_actuator_cmds_plane",minimum_actuator_cmds);
write_value_to_JSON_file("ToConfigurations/general.json","maximum_actuator_cmds_plane",maximum_actuator_cmds);

write_value_to_JSON_file("ToConfigurations/general.json","minimum_io_range_plane",minimum_io_range);
write_value_to_JSON_file("ToConfigurations/general.json","maximum_io_range_plane",maximum_io_range);

% Pre-defined
write_value_to_JSON_file("ToConfigurations/general.json","yaw_saturation",[6.00,-6.00,0.0]);
write_value_to_JSON_file("ToConfigurations/general.json","act_allocation","TriCopterKU_vtol_manual_control");
write_value_to_JSON_file("ToConfigurations/general.json","running_target","RPI_UBUNTU20");

%% Write to json - Allocation Json
write_value_to_JSON_file("ToConfigurations/Allocation_types/general.json","TriCopterKU_vtol_manual_control_gain_positive",TriCopterKU_vtol_manual_control_gain_positive);
write_value_to_JSON_file("ToConfigurations/Allocation_types/general.json","TriCopterKU_vtol_manual_control_gain_negative",TriCopterKU_vtol_manual_control_gain_negative);

write_value_to_JSON_file("ToConfigurations/Allocation_types/general.json","TriCopterKU_vtol_manual_control_plane_gain_positive",TriCopterKU_vtol_manual_control_plane_gain_positive);
write_value_to_JSON_file("ToConfigurations/Allocation_types/general.json","TriCopterKU_vtol_manual_control_plane_gain_negative",TriCopterKU_vtol_manual_control_plane_gain_negative);

write_value_to_JSON_file("ToConfigurations/Allocation_types/general.json","sequence_of_input_commands","[roll,pitch,yaw,throttle,forward,roll_ff, pitch_ff, yaw_ff]");

%% Write to json - Post Motor Allocation Bias Json
write_value_to_JSON_file("ToConfigurations/Allocation_types/PostAllocationMotorBias/general.json","TriCopterKU_vtol_manual_control",TriCopterKU_vtol_manual_control);
write_value_to_JSON_file("ToConfigurations/Allocation_types/PostAllocationMotorBias/general.json","TriCopterKU_vtol_manual_control_plane",TriCopterKU_vtol_manual_control_plane);