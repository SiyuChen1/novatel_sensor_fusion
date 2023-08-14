function convert_ros_timestamp_to_float(ros_ts){
    var seconds = parseInt(ros_ts.nanosec) / 1e9;
    var t = parseInt(ros_ts.sec) + seconds
    return t;
}

function addData(chart, xValue, yValue, index) {
    chart.data.datasets[index].data.push({x: xValue, y: yValue});
    chart.update({
        mode: 'none'
    });
}

let windowStart = 0;

function updateXAxis(chart) {
    windowStart += 10;  // Move the window by 10 seconds
    chart.options.scales.x.min = windowStart;
    chart.options.scales.x.max = windowStart + 10;
    chart.update({
        mode: 'none'
    });
}

L.Control.CustomButton = L.Control.extend({
    options: {
        position: 'topright'
    },

    onAdd: function(map) {
        var button = L.DomUtil.create('button', 'my-custom-button');
        button.innerHTML = 'Toggle Fullscreen';

        L.DomEvent.on(button, 'click', function() {
            var plotDiv = document.getElementById('plot');
            if (plotDiv.style.display === 'none') {
                plotDiv.style.display = 'block';
                button.innerHTML = 'Toggle Fullscreen';
                // console.log('none status');
                map.invalidateSize();
            } else {
                plotDiv.style.display = 'none';
                // console.log('block status');
                button.innerHTML = 'Display Plot';
                map.invalidateSize();
            }
        });

        return button;
    }
});

L.control.customButton = function(opts) {
    return new L.Control.CustomButton(opts);
}

document.addEventListener('DOMContentLoaded', () => {
    // Connect to the server via Socket.IO
    const socket = io();

    var latitude_plot = {
        datasets: [{
            label: '/bestpos',
            data: [],
            borderColor: 'rgba(75, 192, 192, 1)',
            fill: false
        },
        {
            label: '/bestgnsspos',
            data: [],
            borderColor: 'rgba(255, 99, 132, 1)',
            fill: false
        }]
    };

    var options = {
        plugins: {
            title: {
                display: true,
                text: 'Latitude'
            },
            legend: {
                display: true,  // Show or hide the legend.
                position: 'bottom',  // Position of the legend. Can be 'top', 'left', 'bottom', 'right'.
                align: 'center',  // Alignment. Can be 'start', 'center', 'end'.
                labels: {
                    boxWidth: 20,  // Width of legend color box.
                    padding: 10,  // Padding between legend items.
                    color: '#000',  // Font color.
                    font: {
                        size: 14  // Font size.
                    }
                }
            }
        },
        scales: {
            x: {
                type: 'linear',
                position: 'bottom',
                beginAtZero: false,
                title: {
                    display: true,
                    text: 'Time (s)' // x-axis label
                }
            },
            y: {
                beginAtZero: false,
                title: {
                    display: true,
                    text: 'Value' // y-axis label
                }
            }
        },
        maintainAspectRatio: true
};

    // Initialize chart data and settings
    var ctx = document.getElementById('lla_plot').getContext('2d');
    var myLineChart = new Chart(ctx, {
        type: 'line',
        data: latitude_plot,
        options: options,
        responsive: true
    });

    // // Move the x-axis window every 10 seconds
    // setInterval(function() {
    //     updateXAxis(myLineChart);
    // }, 10500);


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

    L.control.customButton().addTo(map);

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

        var now = convert_ros_timestamp_to_float(stamp);
        var duration = now - start;
        addData(myLineChart, duration, latitude, 1);

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
        var duration = now - start;
        addData(myLineChart, duration, latitude, 0);
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

    socket.on('diff_best_bestgnss', (msg)=>{
        console.log(msg.header.frame_id)
        console.log(msg.vector.x)
    })
});