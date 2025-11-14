# Overview
How to create new message definitions and types for DroneLeaf internal use.


## Generating a sample message

Modify in mavlink repo `message_definitions/droneleaf_mav_msgs.xml`

For example, add:
```xml
<message id="77023" name="LEAF_KEY_VAL_FLOAT_PAIR">
    <description>A key-value pair message with float val</description>
    <field type="char[64]" name="key">The key as a character array</field>
    <field type="float" name="val_float">The value of type float</field>
</message>
```

and then at the repo root folder generate for C:

```bash
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=. message_definitions/v1.0/all.xml
```

and for Python (usefult for prototyping and debugging, refer to the mavlink debugging guide):
```bash
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=pymavlink message_definitions/v1.0/all.xml
```

you might wish to update Wireshark plugins for message dissection support, see [Wireshark](./Wireshark/) folder for that.

Make sure to follow these [instructions](https://droneleaf.github.io/petal-app-manager/development_contribution/contribution_guidelines.html#mavlink-pymavlink-very-special-case) to contribute your changes correctly.

## How to use the new message
If you will use it in c++, make sure to update the following files accordingly:
* HEAR_FC/src/HEAR_FC/Flight_controller/HEAR_Util/include/Common/HEARTypetoString.hpp
* HEAR_FC/src/HEAR_FC/Flight_controller/HEAR_Util/src/Common/HEARTypetoString.cpp
* HEAR_FC/src/HEAR_FC/Flight_controller/HEAR_Blocks/include/HEAR_core/System.hpp

### How to map data to the newly created message
For this you will need to create a new Block which maps input data to the new mavlink message. you can use this [code](https://github.com/DroneLeaf/HEAR_Blocks/blob/08594a66516058c8603fe76438f4a5b448beed83/src/HEAR_mavlink/StructureSetpointOffsetMsg.cpp) as an example.

```
auto structure_mavlink_setpoint_offset_msg = new StructureSetpointOffsetMsg();
this->addBlock(structure_mavlink_setpoint_offset_msg, "structure_mavlink_traj_offset_msg");


this->connect(sum_pos_ref->getOutputPort<Vector3D<float>>(), structure_mavlink_setpoint_offset_msg->getInputPort<Vector3D<float>>(StructureSetpointOffsetMsg::IP::OFFSET_POSITION));
this->connect(sum_yaw_ref->getOutputPort<float>(), structure_mavlink_setpoint_offset_msg->getInputPort<float>(StructureSetpointOffsetMsg::IP::OFFSET_YAW));
this->connect(structure_mavlink_setpoint_offset_msg->getOutputPort<mavlink_leaf_setpoint_offset_t>(), mavlink_pub_setpoint_offset->getInputPort<mavlink_leaf_setpoint_offset_t>());
```
