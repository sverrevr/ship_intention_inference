# ship_intention_inference


Going from continuous to discrete:
intention_model_continious.xdsl is used to design the network but must be discretized before use. 
This is done by opening the model in Genie (https://www.bayesfusion.com/) then opening the menu "Network" and pressing Discretize. 
The network must then be made dynamic, this is done by choosing "Network" - "Dynamic Models" - "Enable temporal plate"
All nodes that are supposed to be temporal (all except for intentions nodes) must then be marked and dragged into the temporal plate. 
Special care must be given to the "has turned starboardwards/portwards" node as they depend on their previous state. 
Make a new arc from these nodes to themselves and choose "Order 1" and delete the "has turned starboardwards/portwards t-1" nodes. Open the definition of the "has turned starboardwards/portwards" nodes, for "t = 0" the CPT of the "has turned starboardwards/portwards t-1" in the continuous model node is used, for "t = 1" the CPT of the "has turned starboardwards/portwards" in the continuous model node is used.

Making a model for more ships:
Mark all nodes that should be repeated for multiple ships (these are all noes with ship0 at the end of their name).
Copy and paste these node (within the temporal plate). 
Save the network and open it in sublime text (or another text editing tool). We now need to rename the new nodes from "Copy\_of\_[node\_name]\_ship0" to "[node\_name]\_ship1" (or 2 or 3 etc).
This can be done with the regex expression:
```
Find: Copy_of_(.*?)ship0
Replace: $1ship1
```
And then:
```
Find: Copy of (.*?)ship0
Replace: $1ship1
```
Save and open the file in genie
Next the observation_applicable node must be updated such that both "applicable_or_disabled_ship0" and "applicable_or_disabled_ship1" must be true for it to be true.
Then "stands_on_correct_or_forced_to_give_way_ship0/1" must be updated. For ship 0 it should be true if the ship_stands_on_correct or if ship1 is enabled, it has a gw role to ship1, gives way correct to ship1, and has not safely passed ship1. Similarly for ship1. 