# mobility_games
Projects related to teaching Orientation and Mobility through instrumented games

*please update this readme if you know the status of any of the games in this repository other than Semantic Waypoints!*


## Semantic Waypoints Game
### Overview
The semantic waypoints game is a navigation-based game that involves getting directions to goals (based on april tags) and leaving messages when you arrive for future students to find. This is based on a calibration phase (in which the instructor walks around the area, finding each april tag and labelling it with a name or its location) and a game phase (in which the students set goals, receive verbal directions, and must find the end point). When the student reaches their target destination, a fun sound plays, any previous messages at that location are read aloud to them, and they are able to leave a written message for future students to hear. 

The game is currently built for python+ros on a laptop with a Tango connected via wifi/hotspot, and relies on April Tags printed on sheets of paper and posted to walls or in other locations. The script to modify at is called: `mobility_games/mobility_games/games/semantic_waypoints.py`.

### Installing and Running the Game
Install dependencies and repositories as per [these instructions](https://github.com/occamLab/assistive_apps/blob/summer2018/README_InvisibleMap_Setup.md), but you can leave out anything assistive apps or invisible map specific (e.g. Invisible Map dependencies, cloning the assistive apps repository, etc.), and stop when you reach the portion about installing g2o (an algorithm not used here).

**To run the game:**
1. Connect to your laptop via the tango via the Testing ROS Streamer app on Olin-Robotics. For instructions, see the first bullet under "Running the Invisible Map" [here](https://github.com/occamLab/assistive_apps/blob/summer2018/README_InvisibleMap_Setup.md#running-the-invisible-map).
2. In two separate terminal windows, run the following two commands:
```bash
$  roslaunch tango_streamer stream.launch
$  roslaunch mobility_games semantic_waypoints.launch
```
3. Scan the tango across the area including an april tag, and it will automatically detect it and let you label (using the terminal) where it is.
4. After walking around and scanning multiple tags, you can type `a` in the tiny popped up ROS keyboard window, and the semantic waypoints script will enter run mode. This will prompt you to enter the name of a tag you’ve come across to set as your goal, and once entered, will start giving you directions toward that tag. 

### Troubleshooting
* Make sure you’re running the version on the summer-2018 branch, because we’ve changed a couple things to debug this already.
* If you’re getting a TF_OLD_DATA error referencing ignoring data for odom from /pose_area_server, or really any kind of error regarding odom and TF data, make sure that in tango_streamer’s stream.launch, the node (all ~5 lines or so, surrounded by <node>[...]</node>) including area_learning are commented out or deleted. If they’re not already, do so, and re-run your program.
  * What’s happening here is that pose_area_learning is attempting to be a second parent for the odom frame, which isn’t allowed by tf. If you’re interested in seeing the different between the before and after of this, run (in a separate terminal, while running the other two launch files):
  ```bash
  $ rosrun tf view_frames
  # swing the phone around for a couple seconds while it logs
  $ evince frames.pdf
  ```
  This should pull up a pdf diagram of the nodes, and the diagrams should be different after you delete the area_learning node in stream.launch. You can also use these diagrams to detect other potential issues with the tf heirarchy.


## Musical Cane Game
?

## Grid Game
?

## ?

