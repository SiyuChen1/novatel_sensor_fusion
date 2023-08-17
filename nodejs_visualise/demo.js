'use strict';

const { time } = require('console');
const express = require('express');
const app = express();
const server = require('http').createServer(app);
const io = require('socket.io')(server);
const rclnodejs = require('rclnodejs');

const ParameterType = rclnodejs.ParameterType;
const Parameter = rclnodejs.Parameter;
const ParameterDescriptor = rclnodejs.ParameterDescriptor;

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

  rclnodejs.init().then(() => {
    // create a ros2 node
    const nodeVis = new rclnodejs.Node('visualisation', 'nodejs');

    const topicsInfo = nodeVis.getTopicNamesAndTypes();

    const GetParametersType = 'rcl_interfaces/srv/GetParameters';
    const client = nodeVis.createClient(GetParametersType,
        '/sync/gnss_message_sync/get_parameters');

    const paramNameRequest = {
      names: ['best_topic_name', 'bestgnss_topic_name', 'difference_best_bestgnss', 'difference_best_fused']
    };

    let topicsName = [];

    client.sendRequest(paramNameRequest, (response) => {
        for(let i = 0; i < response.values.length; i++){
            console.log(`Received parameter value: ${response.values[i].string_value}`);
            const topicName = response.values[i].string_value;
            topicsName.push(topicName);
            const foundObject = topicsInfo.find(item => item.name === topicName);
            if (foundObject){
                nodeVis.createSubscription(foundObject.types[0], topicName, (msg) => {
                    //console.log(`Received bestgnsspos message: ${typeof msg}`, msg.latitude);
                    //console.log(JSON.stringify(msg))
                    socket.emit(paramNameRequest.names[i], msg);
                });
            }
            else{
                console.log(`${paramNameRequest.names[i]} not published`);
            }
        }
    });
    // console.log(topicsInfo)

      // parameterClient.get('sync.gnss_message_sync.best_topic_name').then((result) => {
    //   console.log(`Parameter value: ${result}`);
    //   rclnodejs.shutdown();
    // });

    // nodePos.createSubscription('novatel_sensor_fusion/msg/NavSatExtended', '/bestgnss', (msg) => {
    //     //console.log(`Received bestgnsspos message: ${typeof msg}`, msg.latitude);
    //     //console.log(JSON.stringify(msg))
    //     socket.emit('bestgnsspos', msg.latitude, msg.longitude, msg.header.stamp);
    // });
    //
    // nodePos.createSubscription('novatel_sensor_fusion/msg/NavSatExtended', '/best', (msg) => {
    //     // console.log(`Received bestpos message: ${typeof msg}`, msg.latitude);
    //     // console.log(JSON.stringify(msg))
    //     socket.emit('bestpos', msg.latitude, msg.longitude, msg.header.stamp);
    // });
    //
    // nodePos.createSubscription('novatel_sensor_fusion/msg/NavSatExtended', '/fused', (msg) => {
    //     // console.log(`Received bestpos message: ${typeof msg}`, msg.latitude);
    //     // console.log(JSON.stringify(msg))
    //     socket.emit('fusedpos', msg.latitude, msg.longitude, msg.header.stamp);
    // });
    //
    // nodePos.createSubscription('geometry_msgs/msg/Vector3Stamped', '/diff_best_bestgnss', (msg) =>{
    //     // console.log(typeof msg)
    //     socket.emit('diff_best_bestgnss', msg)
    // });

    nodeVis.spin();
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

