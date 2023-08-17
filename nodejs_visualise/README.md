# `nodejs_visualise` package
If a new defined ros message is used in the nodejs, please uninstall `rclnodejs` and reinstall it again in order to generate interface message used for nodejs.
```bash
source ~/catkin_ws/install/setup.bash
npm uninstall rclnodejs
npm install rclnodejs
```
## Example of dynamically modified data using `plotly`
```html
<head>
	<!-- Load plotly.js into the DOM -->
	<script src='https://cdn.plot.ly/plotly-2.24.1.min.js'></script>
</head>

<body>
	<div id='myDiv'><!-- Plotly chart will be drawn inside this DIV --></div>
</body>
```

```javascript
var trace1 = {
  x: [1, 2, 3, 4],
  y: [10, 15, 13, 17],
  type: 'scatter'
};

var trace2 = {
  x: [1, 2, 3, 4],
  y: [16, 5, 11, 9],
  type: 'scatter'
};

var data = [trace1, trace2];

Plotly.newPlot('myDiv', data);

setInterval(function() {
  // Fetch or generate new data here
  var newX = [10, 12];
  var newY = [21, 40];

  // Update the plot with the new data
  Plotly.extendTraces('myDiv', { x: [newX], y: [newY] }, [1]);
}, 1000); 
```