// Copyright (c) 2017 Intel Corporation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

'use strict';

const rclnodejs = require('rclnodejs');

rclnodejs.init().then(() => {
//   const node = rclnodejs.createNode('publisher_example_node');
  const node = new rclnodejs.Node('publisher_example_node');
  const publisher = node.createPublisher('std_msgs/msg/String', 'topic');

  let counter = 0;
  setInterval(() => {
    console.log(`Publishing message: Hello ROS ${counter}`);
    publisher.publish(`Hello ROS ${counter++}`);
  }, 1000);

//   rclnodejs.spin(node);
  node.spin()
});