let windowStart = 0;
let update_time_interval = 5;
let accumulated_error = 0.0;
let accumulated_nb = 0;
let display_plot = true;
let is_start_init = false;

let options = {
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

let latitude_plot = {
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

let north_plot_options = {
    plugins: {
        title: {
            display: true,
            text: 'North'
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

let north_plot = {
    datasets: [{
        label: '/diff_best_bestgnss',
        data: [],
        borderColor: 'rgba(75, 192, 192, 1)',
        fill: false
    },
        {
            label: '/diff_best_fused',
            data: [],
            borderColor: 'rgba(255, 99, 132, 1)',
            fill: false
        }]
};

let east_plot_options = {
    plugins: {
        title: {
            display: true,
            text: 'East'
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

let east_plot = {
    datasets: [{
        label: '/diff_best_bestgnss',
        data: [],
        borderColor: 'rgba(75, 192, 192, 1)',
        fill: false
    },
        {
            label: '/diff_best_fused',
            data: [],
            borderColor: 'rgba(255, 99, 132, 1)',
            fill: false
        }]
};

let up_plot_options = {
    plugins: {
        title: {
            display: true,
            text: 'Up'
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

let up_plot = {
    datasets: [{
            label: '/diff_best_bestgnss',
            data: [],
            borderColor: 'rgba(75, 192, 192, 1)',
            fill: false
        },
        {
            label: '/diff_best_fused',
            data: [],
            borderColor: 'rgba(255, 99, 132, 1)',
            fill: false
        }]
};

function convert_ros_timestamp_to_float(ros_ts){
    const seconds = parseInt(ros_ts.nanosec) / 1e9;
    return parseInt(ros_ts.sec) + seconds;
}

function addData(chart, xValue, yValue, index) {
    chart.data.datasets[index].data.push({x: xValue, y: yValue});
    chart.update({
        mode: 'none'
    });
}

function updateXAxis(chart) {
    chart.options.scales.x.min = windowStart;
    chart.options.scales.x.max = windowStart + 5;
    chart.update({
        mode: 'none'
    });

    for (let i = 0; i < length; i++){
        // clear array and allow garbage collection
        chart.data.datasets[index].data = []
    }
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
                map.invalidateSize();
                display_plot = true;
            } else {
                plotDiv.style.display = 'none';
                button.innerHTML = 'Display Plot';
                map.invalidateSize();
                display_plot = false;
            }
        });

        return button;
    }
});

L.control.customButton = function(opts) {
    return new L.Control.CustomButton(opts);
}

L.Control.Label = L.Control.extend({
    onAdd: function(map) {
        // Create a div element for the label
        this._div = L.DomUtil.create('div', 'label-control');
        this.update(0, 0);
        return this._div;
    },

    // Update the label's content
    update: function(rms_value, nb) {
        this._div.innerHTML = '<h4>Statistical</h4>';
        this._div.innerHTML += `<i>RMS Value = ${rms_value}</i><br>`;
        this._div.innerHTML += `<i>Count of received values = ${nb}</i><br>`;
    }
});

document.addEventListener('DOMContentLoaded', () => {
    // Connect to the server via Socket.IO
    const socket = io();

    // Initialize chart data and settings
    let ctx = document.getElementById('lla_plot').getContext('2d');
    let myLineChart = new Chart(ctx, {
        type: 'line',
        data: latitude_plot,
        options: options,
        responsive: true
    });

    // Initialize chart data and settings
    let north_ctx = document.getElementById('north_plot').getContext('2d');
    let north_chart = new Chart(north_ctx, {
        type: 'line',
        data: north_plot,
        options: north_plot_options,
        responsive: true
    });

    // Initialize chart data and settings
    let east_ctx = document.getElementById('east_plot').getContext('2d');
    let east_chart = new Chart(east_ctx, {
        type: 'line',
        data: east_plot,
        options: east_plot_options,
        responsive: true
    });

    // Initialize chart data and settings
    let up_ctx = document.getElementById('up_plot').getContext('2d');
    let up_chart = new Chart(up_ctx, {
        type: 'line',
        data: up_plot,
        options: up_plot_options,
        responsive: true
    });


    // Move the x-axis window every 5 seconds
    setInterval(function() {
        if(is_start_init){
            windowStart += update_time_interval;  // Move the window by 5 seconds
            updateXAxis(myLineChart);
            updateXAxis(north_chart);
            updateXAxis(east_chart);
            updateXAxis(up_chart);
            let rms = Math.sqrt(accumulated_error / accumulated_nb);
            label_rms.update(rms.toFixed(3), accumulated_nb);
            console.log('nb', accumulated_nb)
        }
    }, update_time_interval * 1000);


    let start;
    let nb_cycle = 100;
    let cur_id = 0;
    let cur_polyline, ref_polyline, fused_polyline;
    let ref_lat = 50.77766817103, ref_lon = 6.07832598964;

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

    let label_rms = new L.Control.Label({ position: 'bottomright' });
    label_rms.addTo(map);

    var cur_mark = L.marker([ref_lat, ref_lon]).addTo(map);
    cur_mark.bindPopup('Reference location')
        .openPopup();


    // Listen for geodata updates from the server
    socket.on('bestgnss_topic_name', (msg) => {
        const stamp = msg.header.stamp;
        if (!is_start_init){
            is_start_init = true;
            start = convert_ros_timestamp_to_float(stamp);
        }

        let now = convert_ros_timestamp_to_float(stamp);
        let duration = now - start;
        addData(myLineChart, duration, msg.latitude, 1);

        if (cur_id < nb_cycle){
            cur_polyline.addLatLng([msg.latitude, msg.longitude]);
            cur_id = cur_id + 1;
        }
        if (cur_id == nb_cycle){
            // console.log(cur_time.nanosec)
            
            let cur_time = stamp.sec + String(stamp.nanosec).substring(0, 3)
            cur_time = new Date(parseInt(cur_time)).toISOString()
            let temp_marker = L.marker([msg.latitude, msg.longitude]).addTo(map);
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
    socket.on('best_topic_name', (msg) => {
        const stamp = msg.header.stamp;
        if (!is_start_init){
            is_start_init = true;
            start = convert_ros_timestamp_to_float(stamp);
        }
        // console.log(latitude, longitude)
        ref_polyline.addLatLng([msg.latitude, msg.longitude]);

        let now = convert_ros_timestamp_to_float(stamp);
        let duration = now - start;
        addData(myLineChart, duration, msg.latitude, 0);
    });

    // Listen for geodata updates from the server
    socket.on('difference_best_fused', (msg) => {
        const stamp = msg.header.stamp;
        if (!is_start_init){
            is_start_init = true;
            start = convert_ros_timestamp_to_float(stamp);
        }

        // console.log(latitude, longitude)
        fused_polyline.addLatLng([msg.latitude, msg.longitude]);
    });

    socket.on('difference_best_bestgnss', (msg)=>{
        const stamp = msg.header.stamp;
        if (!is_start_init){
            is_start_init = true;
            start = convert_ros_timestamp_to_float(stamp);
        }

        const now = convert_ros_timestamp_to_float(stamp);
        const duration = now - start;

        console.log('display_plot', display_plot);
        if (display_plot){
            addData(north_chart, duration, Math.abs(msg.vector.y), 0);
            addData(east_chart, duration, Math.abs(msg.vector.x), 0)
            addData(up_chart, duration, Math.abs(msg.vector.z), 0)
        }

        accumulated_error = accumulated_error +
            msg.vector.x * msg.vector.x +
            msg.vector.y * msg.vector.y +
            msg.vector.z * msg.vector.z;
        accumulated_nb += 1;
    })
});