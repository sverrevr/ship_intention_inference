# ship_intention_inference


Going from continuous to discrete:
intention_model_continious.xdsl is used to design the network but must be discretized before use. 
This is done by opening the model in Genie (https://www.bayesfusion.com/) then opening the menu "Network" and pressing Discretize. 
The network must then be made dynamic, this is done by choosing "Network" - "Dynamic Models" - "Enable temporal plate"
All nodes that are supposed to be temporal (all except for intentions nodes) must then be marked and dragged into the temporal plate. 
Special care must be given to the "has turned starboardwards/portwards" node as they depend on their previous state. 
Make a new arc from these nodes to themselves and choose "Order 1" and delete the "has turned starboardwards/portwards t-1" nodes. Open the definition of the "has turned starboardwards/portwards" nodes, for "t = 0" the CPT of the "has turned starboardwards/portwards t-1" in the continuous model node is used, for "t = 1" the CPT of the "has turned starboardwards/portwards" in the continuous model node is used.