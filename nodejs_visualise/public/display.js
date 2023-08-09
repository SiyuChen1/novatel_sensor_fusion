function convert_ros_timestamp_to_float(ros_ts){
    var t = parseInt(ros_ts.sec + String(ros_ts.nanosec).substring(0, 3))
    // console.log('ros timestamp nanosec', ros_ts.nanosec);
    // console.log(String(ros_ts.nanosec).substring(0, 3))
    // console.log('ros timestamp sec', ros_ts.sec);
    // console.log('t', t);
    return t;
}
document.addEventListener('DOMContentLoaded', () => {

    // Connect to the server via Socket.IO
    const socket = io();
    var start;
    var is_start_init = false;
    var nb_cycle = 100;
    var cur_id = 0;
    var cur_polyline, ref_polyline, fused_polyline;
    var ref_lat = 50.77766817103, ref_lon = 6.07832598964;

    // Create a map centered on a default location
    map = L.map('map').setView([ref_lat, ref_lon], 13); // Adjust the zoom level as desired
    // Create a tile layer using OpenStreetMap data
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors',
        maxZoom: 19
    }).addTo(map);

    // https://www.simplifiedsciencepublishing.com/resources/best-color-palettes-for-scientific-figures-and-data-visualizations
    // https://rgbacolorpicker.com/hex-to-rgba
    // dark red: rgba(193, 39, 45, 0.4), light_red: rgba(193, 39, 45, 0.2), green: rgba(0, 129, 118, 1)
    var dark_red = '#c1272d66', light_red = '#c1272d33', green = '#008176';
    cur_polyline = L.polyline([], {color: light_red, weight: 5}).addTo(map);
    ref_polyline = L.polyline([], {color: dark_red, weight: 4}).addTo(map);
    fused_polyline = L.polyline([], {color: green, weight: 4}).addTo(map);

    // https://codepen.io/haakseth/pen/KQbjdO
    /*Legend specific*/
    var legend = L.control({ position: "bottomleft" });

    var initialData = [{
        x: [1, 2, 3, 4],
        y: [5, 6, 7, 8],
        mode: 'lines'
    }];

    var layout = {
        title: 'Real-time Data from rclnodejs',
        xaxis: {
            title: 'Time'
        },
        yaxis: {
            title: 'Value'
        }
    };

    // TESTER = document.getElementById('error_plot');
	// Plotly.newPlot( TESTER, [initialData], layout);

    // LLAPLOT = document.getElementById('lla_plot');
	// Plotly.newPlot(LLAPLOT, initialData, layout);

    legend.onAdd = function(map) {
        var div = L.DomUtil.create("div", "legend");
        div.innerHTML += "<h4>Legend</h4>";
        div.innerHTML += `<i style="background: ${light_red}"></i><span>GNSS Pose</span><br>`;
        div.innerHTML += `<i style="background: ${dark_red}"></i><span>Grund Truth</span><br>`;
        div.innerHTML += `<i style="background: ${green}"></i><span>Fused</span><br>`;
        return div;
    };

    legend.addTo(map);

    var cur_mark = L.marker([ref_lat, ref_lon]).addTo(map);
    cur_mark.bindPopup('Reference location')
        .openPopup();


    // Listen for geodata updates from the server
    socket.on('bestgnsspos', (latitude, longitude, stamp) => {
        if (!is_start_init){
            is_start_init = true;
            start = convert_ros_timestamp_to_float(stamp);
        }
        if (cur_id < nb_cycle){
            cur_polyline.addLatLng([latitude, longitude]);
            cur_id = cur_id + 1;
        }
        if (cur_id == nb_cycle){
            // console.log(cur_time.nanosec)
            
            var cur_time = stamp.sec + String(stamp.nanosec).substring(0, 3)
            cur_time = new Date(parseInt(cur_time)).toISOString()
            var temp_marker = L.marker([latitude, longitude]).addTo(map);
            temp_marker.bindPopup(`Pose at ${cur_time}`)
                .openPopup();
            
            setTimeout(function() {
                temp_marker.remove();
            }, 20000);

            cur_polyline.setStyle({ color: light_red });
            map.fitBounds(cur_polyline.getBounds());
            cur_polyline = L.polyline([], {color: dark_red, weight: 5}).addTo(map);
            cur_id = 0;
        }
    });

    // Listen for geodata updates from the server
    socket.on('bestpos', (latitude, longitude, stamp) => {
        if (!is_start_init){
            is_start_init = true;
            start = convert_ros_timestamp_to_float(stamp);
        }
        // console.log(latitude, longitude)
        ref_polyline.addLatLng([latitude, longitude]);

        var now = convert_ros_timestamp_to_float(stamp);
        // console.log(now)
        // console.log(typeof now)
        // console.log(start)
        // console.log(typeof start)
        // console.log('duration', now - start)
        var duration = (now - start) / 1e3;

        // initialData.at(0).x.push(duration);
        // // console.log(stamp)
        // // console.log(typeof stamp)
        // // console.log(start)
        // initialData.at(0).y.push(longitude);
        // console.log('x', initialData.x)
        // console.log('y', initialData.y)
        // Plotly.react(LLAPLOT, initialData);

    });

    // Listen for geodata updates from the server
    socket.on('fusedpos', (latitude, longitude, stamp) => {
        if (!is_start_init){
            is_start_init = true;
            start = convert_ros_timestamp_to_float(stamp);
        }

        // console.log(latitude, longitude)
        fused_polyline.addLatLng([latitude, longitude]);
    });
});