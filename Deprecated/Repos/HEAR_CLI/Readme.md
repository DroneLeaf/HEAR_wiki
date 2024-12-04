# Overview
## Aim
To automate robotics developers (and beyond) tasks in managing Unix clusters with a native feeling.

## Philosophy
Let a cluster be called {drones_team_A}. Then performing `hear_cli copy -r 'src_directory' 'dst_directory'` with 'dst_directory={drones_team_A}/home/{username}/HEAR_FC/devel/' would copy from `src_directory` to all cluster hosts.

Note that {drones_team_A} is supplied in the `hear_cli copy` command parameters. But `{username}` is supplied through `HEAR_Configurations`. So `hear_cli` has two and only two data sources: command parameters and `HEAR_Configurations`.

For convinience, `hear_cli` commands support 'interactive' and 'typed' command parameters. 'Interactive' command parameters facilitate parameters entry and help avoid errors. 'Typed' commands are just the conventional way of inputting text following certain command.

hear_cli supports aggregate commands specific for robotics use. For example, commands specific to ROS setup for a fleet of robots can be found under the 'ROS' folder.

