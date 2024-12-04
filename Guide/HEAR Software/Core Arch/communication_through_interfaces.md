# Motivation
With robots we have many ways for components and systems to communicate. Interfaces were created to provide isolation of Blocks and Systems from underlying drivers and communication protocols.

# Components
In HEAR, communication is done in a standardized way. Consider a data packet coming through ethernet/serial/websocket etc.:
1. The Packet enters a `Driver`. Drivers are written in HEAR_Util. Drivers communicate with OS and are OS and protocol specific. Drivers communicate with `Interface Controllers` through:
   1. `IOWriter` class for Drivers that write to I/O.
   2. `Caller<T>` class that writes to the interface controllers what we got from I/O.
2. `Interface Controllers` frames/deframes data mediating between drivers and HEAR blocks. `Interface Factory` utilizes the corresponding interface controller to create publishers, subscribers, servers, and clients.
3. Publishers, subscribers, servers, and clients are provided with the unique address of the resource that needs to be addressed.

# Example
For OptiTrack (numeric order corresponds to the above numeric order):
1. The driver takes the IP/Port and listens to data.
2. The interface controller deframes incoming OptiTrack packets and extracts all rigid-bodies.
3. Each subscriber takes a single rigid body id and outputs the pose of the corresponding rigid body.