# Editing LeafMC UI

This guide will walk you through the steps to add/edit a button to LeafMC (QGC), the files that need to be changed, and how to handle MAVLink messages.

## Prerequisites
- Basic knowledge of C++ and QML
- LeafMC source code
- Qt 5.15.2 installed (use `hear-cli local_machine run_program` program `install_qt`) - use `qmake --version` to check currently installed version
- Qt Create (`sudo apt-get install qtcreator`)
- Execute `sudo ./tools/install-dependencies-debian.sh` in LeafMC repo to install LeafMC build dependencies.

## 1. Where to add UI elements
In general, all UI elements are added and implemented in .qml files.
To add a new element to specific view or parent element, mainly we go that view folder and create new .qml or duplicate an existing one or edit an existing element.

Keep in mind, qml files are mainly UI description files, but it can contains logic also (similar to JavaScript).

Now, to Add a new action button to Fly View toolstrip for example we have to follow the below steps:

1. Open the project in Qt Creator (select the CMakelists.txt file in LeafMC) and click Configure.
**Note:** if Configure emits errors, check the Qt Version in the Project Kits is 5.15.2
**Advice:** familiarize yourself with Qt Creator UI.
**Advice:** Use vscode for file editing.
2. Create `MyButton.qml`

    Go to `src/FlightDisplay` and create a new file `MyButton.qml` (or duplicate an existing qml and rename it)
    ```qml
    import QGroundControl.FlightDisplay 1.0

    GuidedToolStripAction {
        property string leafMode: _guidedController._activeVehicle.leafMode

        text:       _guidedController.myButtonTitle // this is the title of the button
        iconSource: "/res/myButtonTitle.svg" // should be 72x72 svg icon
        visible:    true // or use a condition to control visibility ex: leafMode.length > 0
        enabled:    true // or use a condition to control enabled state ex: leafMode.length > 0
        actionID:   _guidedController.actionMyButton // this is the action ID that will be used to handle the action in the guidedController qml
    }
    ```
    Concerns
    - myButtonTitle: where to define?
    - actionMyButton: where to define?
    - "/res/myButtonTitle.svg": how to add the icon?
3. Add MyButton.qml to the FightDisplay CMakelists.txt

    ```cmake
    add_custom_target(FligthDisplayQml
	    SOURCES
        ...
        MyButton.qml
    )
    ```
4. Add MyButton.qml to the qgroundcontrol.qrc

    ```xml
    <qresource prefix="/qml">
        ...
        <file alias="QGroundControl/FlightDisplay/MyButton.qml">src/FlightDisplay/MyButton.qml</file>
    </qresource>
    ```
5. Add MyButton.qml to the action list toolstrip

    Go to src/FlightDisplay/FlyViewToolStripActionList.qml and add the following

    ```qml
    
    import QtQml.Models 2.12

    import QGroundControl           1.0
    import QGroundControl.Controls  1.0

    ToolStripActionList {
        id: _root

        signal displayPreFlightChecklist

        model: [
            ...
            MyButton { }
        ]
    }
    ```
6. Define myButtonTitle and actionMyButton in GuidedActionsController.qml

    Go to src/FlightDisplay/GuidedActionsController.qml and add the following

    ```qml
    ...
    property string myButtonTitle: qsTr("My Button")

    ...
    readonly property int actionInspectSlaps:               39
    readonly property int actionPausePipeline:              40
    readonly property int actionResumePipeline:             41
    readonly property int actionMyButton:                   42 // new actionID
    
    ```
    Note:
    - in confirmAction function, add a new case for the new actionID
    - In executeAction function, add a new case for the new actionID and put the logic to be executed when the button is clicked.
    - use _activeVehicle to call functions from the Vehicle C++ class, ex _activeVehicle.sayMyButton()
7. Add the icon to the resources
    
    - Go to resources folder and paste the icon file (myButtonTitle.svg) there.
    - Add the icon to the qgcresources.qrc file
    ```xml
    <qresource prefix="/res">
        ...
        <file alias="myButtonTitle.svg">resources/myButtonTitle.svg</file>
    </qresource>
    ```
8. Add sayMyButton function to the Vehicle class

    Go to src/Vehicle/Vehicle.h and add the following
    ```cpp
    Q_INVOKABLE void sayMyButton();
    ```
    Go to src/Vehicle/Vehicle.cpp and add the following
    ```cpp
    void Vehicle::sayMyButton()
    {
        _say("My Button Clicked");
    }
    ```

After compiling and running the project, you should see the new button in the Fly View toolstrip and if you click it you will hear a voice syaing "My Button Clicked".

## 2. C++ variables and its binding to QML
Inside the C++ code, we can define variables and functions that can be accessed from QML files. To do so, we have to use the Q_PROPERTY macro to define the variable and the Q_INVOKABLE macro to define the function.

For example, to define a variable in the Vehicle class that can be accessed from QML, we have to do the following:

    1. Define the variable in the Vehicle.h file
        ```cpp
        Q_PROPERTY(QString myVariable READ myVariable WRITE setMyVariable NOTIFY myVariableChanged)
        ```
    2. Define the getter and setter functions in the Vehicle.h file
        ```cpp
        QString myVariable() const;
        void setMyVariable(const QString &myVariable);
        ```
    3. Define the variable in the Vehicle.cpp file
        ```cpp
        QString Vehicle::myVariable() const
        {
            return _myVariable;
        }

        void Vehicle::setMyVariable(const QString &myVariable)
        {
            if (_myVariable == myVariable)
                return;

            _myVariable = myVariable;
            emit myVariableChanged(_myVariable);
        }
        ```
    4. Add the variable to the Vehicle class constructor in the Vehicle.cpp file
        ```cpp
        _myVariable = "Hello World";
        ```
    5. Define the signal in the Vehicle.h file
        ```cpp
        void myVariableChanged(const QString &myVariable);
        ```
Now in the qml file, we can access the variable using the following code:
    ```qml
        ...
        text: _activeVehicle.myVariable
        ...
    ```

For function it's much simpler, just define the function in the Vehicle.h file using the Q_INVOKABLE macro and implement it in the Vehicle.cpp file.

    1. Define the function in the Vehicle.h file
        ```cpp
        Q_INVOKABLE void sayHello();
        ```
    2. Implement the function in the Vehicle.cpp file
        ```cpp
        void Vehicle::sayHello()
        {
            _say("Hello World");
        }
        ```
Now in the qml file, we can call the function using the following code:
    ```qml
        ...
        _activeVehicle.sayHello()
        ...
    ```

## 3. Handling Mavlink Messages
To handle MAVLink messages, the main logic that capture the messeges or send them is located in the Vhicel class. The Vehicle class is the main class that handles the communication with the vehicle and the MAVLink messages.

### 3.1 Recieve
In Vehicle::_mavlinkMessageReceived function, we can capture the incoming MAVLink messages and handle them accordingly. The function takes a mavlink_message_t struct as a parameter, which contains the message data.
    ```cpp
        Vehicle::_mavlinkMessageReceived(..){
            ...

            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    // handle heartbeat message
                    break;
                case MAVLINK_MSG_ID_SYS_STATUS:
                    // handle system status message
                    break;
                // ADD a case for the new message ID and call the function you want to call based on that.
                // Check other _handleLeaf* methods for examples.
                ...
            }
        }
    ```
### 3.2 Send
To send MAVLink messages, we can use the sendMessageOnLinkThreadSafe function, just define a function that you want it to send a message when it's called and use it inside the function.

    ```cpp
        void Vehicle::sendMyMessage()
        {
            mavlink_message_t msg;
            mavlink_msg_my_message_pack(_systemId, _componentId, &msg, _myVariable);
            sendMessageOnLinkThreadSafe(sharedLink.get(), msg);
        }
    ```
## 4. Leaf Modes
LeafMC has different modes that the vehicle can be in, each mode has its own set of actions and behaviors.

To add or edit the modes list check the Vehicle::Vehicle(..) constructor where the list of modes.

To make the available modes conditioned based on some logic, edit the code in the Vehicle::leafModes() function.

## 5. Leaf Status
LeafMC has different status that the vehicle can be in, populated by LeafFC.
Same as the modes, the status list is defined in the Vehicle::Vehicle(..) constructor.

To add or edit the status list check the Vehicle::Vehicle(..) constructor where the list of status.

## Conclusion
This guide should give you a good starting point to add/edit UI elements in LeafMC and handle MAVLink messages. For more information, you can refer to the QGroundControl documentation and the LeafMC source code.
