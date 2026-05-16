behaviour described:
in the simulation: 9 drones, connect into 3 clusters of 3
withinthe cluster:
- drones become neightbors, each knows id of its neightbors and tracks dheir most recent positions and time when they seen them there,
- in the constant time intrvals drones broadcast messages as described here:
 messages are send using c subprocesses, pattern matching is as in the class.

so, each time interval specified in processes mesage is send. time is number of processes in te backgroud. the messages are written to the file, each node reads from the file and matches patters (we only ignore ours sends). if it is normal alive ping, we update the position of the neightbor with this id into what we received and last time seen to current time. if we receive drone missing, we should respond with when did we seen it for last time *send last_seen. the other side computes for how long the missing drone was missing. if it is long enough, then we announce missing (not implemented for now), if he concludes that the time is not sufficient, then we just continue normally

and remember, we keep track only of our two neightbours.

I woud like to implement this for now. could you help me? keep as much from te code I have so far as possible 