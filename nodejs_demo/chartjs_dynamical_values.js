document.addEventListener('DOMContentLoaded', () => {
    // Initialize chart data and settings
    var ctx = document.getElementById('myLineChart').getContext('2d');

    var data = {
        datasets: [{
            label: 'Line 1',
            data: [],
            borderColor: 'rgba(75, 192, 192, 1)',
            fill: false
        },
        {
            label: 'Line 2',
            data: [],
            borderColor: 'rgba(255, 99, 132, 1)',
            fill: false
        }]
    };

    var options = {
        plugins: {
            title: {
                display: true,
                text: 'My Chart Title'
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
                beginAtZero: true,
                title: {
                    display: true,
                    text: 'Time (s)' // x-axis label
                }
            },
            y: {
                beginAtZero: true,
                title: {
                    display: true,
                    text: 'Value' // y-axis label
                }
            }
        }
    };


    var myLineChart = new Chart(ctx, {
        type: 'line',
        data: data,
        options: options
    });

    // Utility functions to add and generate data
    function getRandomValue() {
        return Math.floor(Math.random() * 60) + 10;  // returns a random number between 10 and 70
    }

    function addData(chart, xValue, data1, data2) {
        chart.data.datasets[0].data.push({x: xValue, y: data1});
        chart.data.datasets[1].data.push({x: xValue, y: data2});
        chart.update();
    }

    // Add data every 5 seconds
    let currentTime = 0;

    setInterval(function() {
        currentTime += 1;
        addData(myLineChart, currentTime, getRandomValue(), getRandomValue());
    }, 1000);

});
