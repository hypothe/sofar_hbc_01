
# Next steps

## 1. tf_server

reads from /tf topic and:
 1a. publish msg with human tf 	(/tf/human) [from /tf , have "frame_id: world" and "children_frame_id:" to something beside "base", block ids and the like]
 1b. publish msg with robot tf 	(/tf/baxter) [from /tf, have "frame_id:" different from "world" except for (frame_id: world; children_frame_id: base)]
 1c. server for blocks tf 		(/tf/blocks) [from /tf, "frame_id: world" and "child_frame_id:" either a char, "Bluebox", "Redbox", "MiddlePlacement", "MiddlePlacementN"]

## 2. check_collision

probably implements FCL interface (https://github.com/flexible-collision-library/fcl) (one for each arm)
 2a. gather data from /tf/human -> make cylinders
 2b. gather data from /tf/baxter -> make cylinders
 2c. check collision
 2d. if a collision is detected publish a msg on /baxter_moveit_stop_trajectory (modify the current msg by adding "str arm" as field).
	*NOTE: we need to modify Unity Baxter Controller script; once this message is received by Unity it will look at the "arm" field 
	of it, see if a coroutine for that process is running and then terminate it. A coroutine pointer shall be saved in a variable (as it
	already is), one for each arm, so that it can be easily traced and stopped.

## 3. closest_block

 3a. implement a server for "block_to_pick" request, with current eef pose (geometry_msgs::Pose) and arm (string)
 3b. when the request is received it makes itself a request to /tf_server for /tf/blocks
 3c. check the closest block to the arm pose, between those in the correct half-table (depending on arm designed space)
		Keep in mind that, in case the arm specified is the left one and a block is in the MiddlePlacementN (how to track that? Parameter Server?)
		that block should take precedence.

## 4. closest_empty_space

 4a. implement a server for /empty_pos, with block_pos field (geometry_msgs::Pose)
 4b. randomly sample a point around that pos in some way:
	- to make things easy let's assume in a semi-circle: randomly select the radius between 5 and 10 cm and the angle between pi and 2*pi for
	the blocks in the right side of the table (notice the world frame is oriented with x pointing from bxtr to the human, y to the left of bxtr, z up)
	or between 0 and pi if it's in the left side (with this semi circle we are guaranteed to move the block away from the midpoint).
 4c. if that position is unobstructed (no block is too close to it) return it, else go back to 4b.
	- we can, as a further step, perhaps think about "smart storage" of the blocks in some structure that guarantees faster retrieval times for
	checking distance relative to a query point. No idea about that for the moment, not really necessary right now.
	
## 5. FiniteStateMachine

two of them, actually the same code but run as two different nodes with ARM as a parameter.
 5a. TODO: fully fledged explanation of it
