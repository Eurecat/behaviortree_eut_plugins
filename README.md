# BehaviorTree EUT Plugins

This library witholds a small set of additional functionalities to be able to:
- Monitor
- Debug
- Replay

Behavior Trees implemented with the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/4.6.2) library (tested with **version 4.6.2**).

## How to compile it (ROS2)

At the moment the package compiles and has been tested with **ROS2 humble**, therefore compile using `colcon build`(ROS2) and making sure public [BehaviorTree.CPP v4.6+](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/4.6.2) has been compiled and/or sourced correctly in the workspace.

## How to use it (ROS2)

For monitoring and debugging the tree just include the following headers:
```
#include "behaviortree_eut_plugins/eut_debug.h"
#include "behaviortree_eut_plugins/loggers/bt_zmq_publisher.h"
```

Create the `tree` with built-in functionalities of the libraries, such as `createTreeFromFile(const std::filesystem::path& file_path, Blackboard::Ptr blackboard)`, making sure it is allocated dynamically to memory and therefore witholding a `shared_ptr` to it.

```
std::shared_ptr<BT::Tree> tree_ptr = std::make_shared<BT::Tree>(factory.createTreeFromFile(tree_file_path));
BT::DebuggableTree debugTree{tree_ptr};

BT::Tree& tree = *(tree_ptr); // if you wanna have a direct ref. to the object
```


If you want to enable monitoring, just instantiate a ZMQPublisher and then you can just tick your tree as you normally do:

```
// Connect the PublisherZMQ. This will allow GrootEUT to
// get the tree and poll status updates.
const unsigned port = 1667;
BT::PublisherZMQ publisher(debugTree, port);

...

  while(1)
  {
    tree.tickWhileRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
```

Monitor and debug your tree through the monitoring functionality available in [Groot humble branch](https://ice.eurecat.org/gitlab/robotics-automation/groot/-/tree/humble). 

## Additional details you might be interested to

### DebuggableTree options

Note that this is the constructor for `DebuggableTree`

```
DebuggableTree(std::shared_ptr<Tree> tree, bool pausable = true, bool start_paused = true)
```

In details:
- `start_paused` it's a flag to pause before the first tick to the root, allowing you the possibility to play the tree directly from Groot
- `pausable` it's a flag to allow for the tree to be interacted with at run-time affecting its execution (pause/resume, breakpoints, enable, WoZ)

### Treat custom types

Assuming you have your custom type Position2D:

```
struct Position2D
{
  double x;
  double y;
};
```

You need to define the macro to generate the code that is needed to convert the object to/from JSON
```
BT_JSON_CONVERTER(Position2D, pos)
{
  add_field("x", &pos.x);
  add_field("y", &pos.y);
}
```

You still need to call `BT::RegisterJsonDefinition<Position2D>()` before instantiating the tree and use monitoring, otherwise object cannot be serialized and sent to the monitoring tool (e.g. Groot).
```
BT::RegisterJsonDefinition<Position2D>();
```

### Examples

I invite you to take a look at the examples under `src/examples`(ROS2) for more details.

If you're stucked or whatever issue arise, please ping Devis (devis.dalmoro@eurecat.org) or David (david.calero@eurecat.org) to avoid wasting any time on it