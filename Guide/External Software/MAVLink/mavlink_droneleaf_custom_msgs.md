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
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=. message_definitions/droneleaf_mav_msgs.xml
```

and for Python (usefult for prototyping and debugging, refer to the mavlink debugging guide):
```bash
python3 -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=pymavlink message_definitions/droneleaf_mav_msgs.xml
```

you might wish to update Wireshark plugins for message dissection support, see Wireshark folder for that.