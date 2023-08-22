let windowStart = 0;
let update_time_interval = 5;
let accumulated_error = 0.0;
let accumulated_nb = 0;
let display_plot = true;
let is_start_init = false;
let display_ground_truth = true;
let display_raw_gnss = true;
let display_fused = true;
const polyline_width = 4;

// https://www.simplifiedsciencepublishing.com/resources/best-color-palettes-for-scientific-figures-and-data-visualizations
// https://rgbacolorpicker.com/hex-to-rgba
// dark red: rgba(193, 39, 45, 0.4), light_red: rgba(193, 39, 45, 0.2), green: rgba(0, 129, 118, 1)
const dark_red = '#c1272d66', light_red = '#c1272d33', green = '#008176';
const coral = '#FF7F5077', teal = '#008080', goldenrod = '#DAA52099';

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

function convert_ros_timestamp_to_float(ros_ts) {
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
    chart.options.scales.x.max = windowStart + update_time_interval;
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
        this._div.innerHTML += `<i>RMS Value = ${rms_value} m</i><br>`;
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
            // console.log('nb', accumulated_nb)
        }
    }, update_time_interval * 1000);


    let start;
    let nb_cycle = 100;
    let cur_id = 0;
    let cur_polyline, ref_polyline, fused_polyline;
    let cur_decorator, ref_decorator, fused_decorator;
    let ref_lat = 50.77766817103, ref_lon = 6.07832598964;

    // Create a map centered on a default location
    map = L.map('map').setView([ref_lat, ref_lon], 13); // Adjust the zoom level as desired

    // Create a tile layer using OpenStreetMap data
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors',
        maxZoom: 19
    }).addTo(map);

    L.control.customButton().addTo(map);

    cur_polyline = L.polyline([], {color: coral, weight: polyline_width}).addTo(map);
    ref_polyline = L.polyline([], {color: teal, weight: polyline_width}).addTo(map);
    fused_polyline = L.polyline([], {color: goldenrod, weight: polyline_width}).addTo(map);

    // https://codepen.io/haakseth/pen/KQbjdO
    /*Legend specific*/
    let legend = L.control({ position: "bottomleft" });

    legend.onAdd = function(map) {
        let div = L.DomUtil.create("div", "legend");
        div.innerHTML += "<h4>Legend</h4>";

        // GNSS Pose
        let gnssPose = L.DomUtil.create("div", "", div);
        gnssPose.innerHTML = `<i style="background: ${cur_polyline.options.color}"></i><span>GNSS Pose</span>`;
        gnssPose.addEventListener("click", function() {
            // console.log("GNSS Pose clicked");
            // Your callback function here
            display_raw_gnss = !display_raw_gnss;
            if (display_raw_gnss){
                this.style.opacity = 1;
                map.addLayer(cur_polyline);
            } else{
                this.style.opacity = 0.5;
                map.removeLayer(cur_polyline);
            }
        });

        // Ground Truth
        let groundTruth = L.DomUtil.create("div", "", div);
        groundTruth.innerHTML = `<i style="background: ${ref_polyline.options.color}"></i><span>Ground Truth</span>`;
        groundTruth.addEventListener("click", function() {
            // console.log("Ground Truth clicked");
            // Your callback function here
            display_ground_truth = !display_ground_truth;
            if (display_ground_truth){
                this.style.opacity = 1;
                map.addLayer(ref_polyline);
            } else{
                this.style.opacity = 0.5;
                map.removeLayer(ref_polyline);
            }
        });

        // Fused
        let fused = L.DomUtil.create("div", "", div);
        fused.innerHTML = `<i style="background: ${fused_polyline.options.color}"></i><span>Fused</span>`;
        fused.addEventListener("click", function() {
            // console.log("Fused clicked");
            // Your callback function here
            display_fused = !display_fused;
            if (display_fused){
                this.style.opacity = 1;
                map.addLayer(fused_polyline);
            } else{
                this.style.opacity = 0.5;
                map.removeLayer(fused_polyline);
            }
        });

        return div;
    };

    legend.addTo(map);

    let label_rms = new L.Control.Label({ position: 'bottomright' });
    label_rms.addTo(map);

    let cur_mark = L.marker([ref_lat, ref_lon]).addTo(map);
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
            }, 5000);

            // cur_polyline.setStyle({ color: coral });
            map.fitBounds(cur_polyline.getBounds());
            // cur_polyline = L.polyline([], {color: coral, weight: 4}).addTo(map);
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

        // // console.log(latitude, longitude)
        // fused_polyline.addLatLng([msg.latitude, msg.longitude]);
        if (display_plot){
            const now = convert_ros_timestamp_to_float(stamp);
            const duration = now - start;
            addData(north_chart, duration, Math.abs(msg.vector.y), 1);
            addData(east_chart, duration, Math.abs(msg.vector.x), 1)
            addData(up_chart, duration, Math.abs(msg.vector.z), 1)
        }
    });

    socket.on('imu_ekf_fused_topic_name', (msg) => {
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

        if (display_plot){
            const now = convert_ros_timestamp_to_float(stamp);
            const duration = now - start;
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