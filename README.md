
# Next steps

## 1. tf_server

> **Priority:**		5/5  
> **Complexity:** 	1/5

reads from /tf topic and:

1. publish msg with human tf 	(/tf/human) [from /tf , have "frame_id: world" and "children_frame_id:" to something beside "base", block ids and the like]
2. publish msg with robot tf 	(/tf/baxter) [from /tf, have "frame_id:" different from "world" except for (frame_id: world; children_frame_id: base)]
3. server for blocks tf 		(/tf/blocks) [from /tf, "frame_id: world" and "child_frame_id:" either a char, "Bluebox", "Redbox", "MiddlePlacement", "MiddlePlacementN"]

## 2. check_collision

> **Priority:**		1/5  
> **Complexity:** 	3/5

probably implements FCL interface (https://github.com/flexible-collision-library/fcl) (one for each arm)

1. gather data from /tf/human -> make cylinders
2. gather data from /tf/baxter -> make cylinders
3. check collision
4. if a collision is detected publish a msg on /baxter_moveit_stop_trajectory (modify the current msg by adding "str arm" as field).

## 3. closest_block

> **Priority:**		4/5  
> **Complexity:** 	4/5

1. implement a server for "block_to_pick" request, with current eef pose (geometry_msgs::Pose) and arm (string)
2. when the request is received it makes itself a request to /tf_server for /tf/blocks
3. Remove from those blocks the one already placed in the Redbox and Bluebox (save them in the param server after their placement )
4. check the closest block to the arm pose, between those in the correct half-table (depending on arm designed space)
		Keep in mind that, in case the arm specified is the left one and a block is in the MiddlePlacementN (how to track that? Parameter Server?)
		that block should take precedence; at the same time, if the blue block is obstructed by a red one, this should be removed first (but trying
		to delay picking it, low priority).

## 4. closest_empty_space

> **Priority:**		3/5  
> **Complexity:** 	3/5

1. implement a server for /empty_pos, with block_pos field (geometry_msgs::Pose)
2. randomly sample a point around that pos in some way:
	- to make things easy let's assume in a semi-circle: randomly select the radius between 5 and 10 cm and the angle between pi and 2*pi for
	the blocks in the right side of the table (notice the world frame is oriented with x pointing from bxtr to the human, y to the left of bxtr, z up)
	or between 0 and pi if it's in the left side (with this semi circle we are guaranteed to move the block away from the midpoint).
3. if that position is unobstructed (no block is too close to it) return it, else go back to 4b.
	- we can, as a further step, perhaps think about "smart storage" of the blocks in some structure that guarantees faster retrieval times for
	checking distance relative to a query point. No idea about that for the moment, not really necessary right now.
	
## 5. FiniteStateMachine

> **Priority:**		3/5  
> **Complexity:** 	4/5

two of them, actually the same code but run as two different nodes with ARM as a parameter.

1. **TODO:** fully fledged explanation of it
2. **TODO:** on Unity side a message sbould be published once a trajectory has been terminated, so that the FSM waits for the current state to be executed before passing to the next one. Also, set a block as "delivered" on the parameter server, so that it will not be checked for ditance and occlusion by the other nodes.

---

# Useful resources

For the retrieval of the closest block to either the current eef pose or to a random empty spot we could improve performances with a [k-d-tree](https://en.wikipedia.org/wiki/K-d_tree); however, the gain in terms of performances at this scale could be negligible so a linear search should suffice for now. If there's time to refine the project consider this option.

For collision checking see the tf tree with `rosrun rqt_tf_tree tqt_tf_tree`, we could probably care only for the links from "xxx_upper_forearm" downward (skipping also few links which only rotate wrt previous ones, eg grouping all hand joints together).
