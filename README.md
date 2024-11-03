# ri

I have fiddled the whole day to try to make the shit work, but only minor successes. No actual working solution where 2 robots can be displayed.

It seems to be an issue of multiple /rviz nodes being present and therefore maybe being the cause of the following infinite deletion loop:
``
[flatland_server-1] [WARN] [1730593307.672788791] [DebugVis]: Deleting topic models/m_robot_1
[flatland_server-1] [WARN] [1730593307.673027084] [DebugVis]: Deleting topic models/m_robot_2
```

Maybe you guys can figure somethin out, but I am done for today.
