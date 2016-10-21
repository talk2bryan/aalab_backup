This is a sprinting module for the kid size ROBOTIS OP2 robot (http://en.robotis.com/index/product.php?cate_code=111310). The objective is for the robot to walk as fast as it can to a given target, and backwards. It uses OPENCV to process images from the camera feed. The purpose of this is for competitions like ROBOCUP and FIRA.

The new feature here is: given two adjacent colors, get the shortest distance between these two, get the point, and that will be the new co-ordinate of the target. The reasoning behind this is that someone can have the same color of the target from the previous version, so, if we have two colors as a working target and derive the shortest distance between all of these colors at i, that would most likely be our target.

Although, if we have someone this occurence elsewhere, say someone wearing the same color pattern on his shirt, then we are screwed -- this will be taken care of later.

By doing this this way, we have a larger target and this can be viewed properly from a farther distance
