

%% Allocation json files

% sequence_of_input_commands [roll,pitch,yaw,throttle,forward,roll_ff, pitch_ff, yaw_ff]

% Plane:
TriCopterKU_vtol_manual_control_plane_gain_positive=[
        [ 0.0           ,  0.0              ,0.0    , 1.0           ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0           ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0           ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0           ,0.0    ,0.0    s4_alloc_pos_gain_plane,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0           ,0.0    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0           ,s8_alloc_gain_pos_plane,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0       ,0.0                    ,-s13_alloc_gain_neg,s13_alloc_gain_pos,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0       ,0.0                    ,s14_alloc_gain_pos ,s14_alloc_gain_pos,0.0]];

TriCopterKU_vtol_manual_control_plane_gain_negative=[
        [ 0.0           ,  0.0              ,0.0    , 1.0           ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0           ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    , 0.0           ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                    ,0.0    ,s4_alloc_neg_gain_plane,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,0.0                    ,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0    ,0.0            ,s8_alloc_gain_neg_plane,0.0    ,0.0    ,0.0];
        [ 0.0           ,  0.0              ,0.0,0.0            ,0.0                    ,-s13_alloc_gain_pos,s13_alloc_gain_neg,0.0];
        [ 0.0           ,  0.0              ,0.0,0.0            ,0.0                    ,s14_alloc_gain_neg ,s14_alloc_gain_neg,0.0]];


