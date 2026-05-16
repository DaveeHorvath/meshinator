behaviour described:
in the simulation: 9 drones, connect into 3 clusters of 3
withinthe cluster:
- drones become neightbors, each knows id of its neightbors and tracks dheir most recent positions and time when they seen them there,
- in the constant time intrvals drones broadcast messages as described here:
 messages are send using c subprocesses, pattern matching is as in the class.

so, each time interval specified in processes mesage is send. time is number of processes in te backgroud. the messages are written to the file, each node reads from the file and matches patters (we only ignore ours sends). if it is normal alive ping, we update the position of the neightbor with this id into what we received and last time seen to current time. if we receive drone missing, we should respond with when did we seen it for last time *send last_seen. the other side computes for how long the missing drone was missing. if it is long enough, then we announce missing (not implemented for now), if he concludes that the time is not sufficient, then we just continue normally

ok, so now it works. now next step. ow what we do when we receive that some dron is lost. if I'm this drone, I'll ignore it (as in real life I would never receive it, I'm out of range), but each other node in the cluster should compute the predcisted position of this drone based on the previous recorded positions (we can for now assume that drones move with linear motion) each drone should calculate it separetely. then they'll exchange this informations (implemented in c, let's say that for now message will be receise position_lost), so that each node computes the average predicted position and which node from their remaining neightbours is the closest one to to this position and for now just print this position.