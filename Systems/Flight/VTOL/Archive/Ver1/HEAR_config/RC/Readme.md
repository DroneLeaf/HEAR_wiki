The map takes from [x1,x2,y1,y2]. x1 and x2 would correspond to RC input limits. y1 and y2 would correspond to the output limit. The mapping is assumed linear in HEAR_FC
To disable an RC input put y1 and y2 to zero. For example disable yaw reference and enable yaw_rate reference by:
    "map_for_yaw":[1150,1850,0.0,0.0]
    "map_for_yaw_rate":[1150,1850,-1.5,1.5]