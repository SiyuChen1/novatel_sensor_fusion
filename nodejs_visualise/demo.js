'use strict';

const { time } = require('console');
const express = require('express');
const app = express();
const server = require('http').createServer(app);
const io = require('socket.io')(server);
const fs = require('fs')
const yaml = require('js-yaml')
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

  rclnodejs.init().then(() => {
    // create a ros2 node
    const nodeVis = new rclnodejs.Node('visualisation', 'nodejs');
    const paramNameRequest = {
      names: ['best_topic_name', 'bestgnss_topic_name', 'imu_ekf_fused_topic_name',
          'difference_best_bestgnss', 'difference_best_fused']
    };
    const topicsInfo = nodeVis.getTopicNamesAndTypes();

    /* Method 1: get topic name via ros2 service */
/*  const GetParametersType = 'rcl_interfaces/srv/GetParameters';
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
                    socket.emit(paramNameRequest.names[i], msg);
                });
            }
            else{
                console.log(`${paramNameRequest.names[i]} not published`);
            }
        }
    });*/

      /* Method 2: get topic name from reading config file */
      const configPath = '../config/params.yaml';
      try {
          const fileContents = fs.readFileSync(configPath, 'utf8');
          const data = yaml.load(fileContents);
          paramNameRequest.names.forEach((key) => {
              const topicName = data['/**']['ros__parameters'][key];
              const foundObject = topicsInfo.find(item => item.name === topicName);
              if (foundObject){
                  nodeVis.createSubscription(foundObject.types[0], topicName, (msg) => {
                      socket.emit(key, msg);
                  });
              }
              else{
                  console.log(`Topic ${topicName} not published`);
              }
          });
      } catch (e) {
          console.error('Error reading or parsing the YAML file:', e);
      }

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

