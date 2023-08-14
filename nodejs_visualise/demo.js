'use strict';

const { time } = require('console');
const express = require('express');
const app = express();
const server = require('http').createServer(app);
const io = require('socket.io')(server);
const rclnodejs = require('rclnodejs');

// Serve static files from the "public" directory
app.use(express.static('public'));

// Start the server
const port = 8889;
server.listen(port, () => {
  console.log(`Server is running on http://localhost:${port}`);
});

// Configure Socket.IO
io.on('connection', (socket) => {
  console.log('Client connected');

  rclnodejs.
  init().
  then(() => {
    // create a ros2 node
    const nodePos = new rclnodejs.Node('subscription_gnsspos_node');

    nodePos.createSubscription('novatel_sensor_fusion/msg/NavSatExtended', '/bestgnss', (msg) => {
        //console.log(`Received bestgnsspos message: ${typeof msg}`, msg.latitude);
        //console.log(JSON.stringify(msg))
        socket.emit('bestgnsspos', msg.latitude, msg.longitude, msg.header.stamp);
    });

    nodePos.createSubscription('novatel_sensor_fusion/msg/NavSatExtended', '/best', (msg) => {
        // console.log(`Received bestpos message: ${typeof msg}`, msg.latitude);
        // console.log(JSON.stringify(msg))
        socket.emit('bestpos', msg.latitude, msg.longitude, msg.header.stamp);
    });

    nodePos.createSubscription('novatel_sensor_fusion/msg/NavSatExtended', '/fused', (msg) => {
        // console.log(`Received bestpos message: ${typeof msg}`, msg.latitude);
        // console.log(JSON.stringify(msg))
        socket.emit('fusedpos', msg.latitude, msg.longitude, msg.header.stamp);
    });

    nodePos.createSubscription('geometry_msgs/msg/Vector3Stamped', '/diff_best_bestgnss', (msg) =>{
        console.log(typeof msg)
        socket.emit('diff_best_bestgnss', msg)
    });

    nodePos.spin();
  }).
  catch((e) => {
        console.log(e)
  });

  // Disconnect event
  socket.on('disconnect', () => {
        console.log('Client disconnected');
        rclnodejs.shutdownAll();
        console.log("ROS2 Node shut down")
  });
});

process.on('SIGINT', function() {
    console.log( "\nGracefully shutting down from SIGINT (Ctrl-C)" );
    // some other closing procedures go here
    process.exit(0);
});

