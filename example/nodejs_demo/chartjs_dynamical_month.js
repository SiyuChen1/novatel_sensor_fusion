function addData(chart, label, data) {
    chart.data.labels.push(label);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(data);
    });
    chart.update();
}

function removeData(chart) {
    chart.data.labels.pop();
    chart.data.datasets.forEach((dataset) => {
        dataset.data.pop();
    });
    chart.update();
}

function getRandomSalesValue() {
    return Math.floor(Math.random() * 60) + 10;  // returns a random number between 10 and 70
}

document.addEventListener('DOMContentLoaded', () => {
  var ctx = document.getElementById('myLineChart').getContext('2d');

    var data = {
        labels: ['January', 'February', 'March', 'April', 'May', 'June', 'July'],
        datasets: [{
            label: 'Monthly Sales',
            data: [10, 20, 15, 35, 25, 50, 40],
            borderColor: 'rgba(75, 192, 192, 1)',
            fill: false
        }, {
            label: 'Monthly Benefit',
            data: [30, 40, 35, 15, 45, 20, 50],
            borderColor: 'rgba(175, 12, 192, 1)',
            fill: false
        }]
    };

    var options = {
        scales: {
            y: {
                beginAtZero: true
            }
        }
    };

    var myLineChart = new Chart(ctx, {
        type: 'line',
        data: data,
        options: options
    });

    let months = ['January', 'February', 'March', 'April', 'May', 'June', 'July', 'August', 'September', 'October', 'November', 'December'];
    let currentMonthIndex = 7; // starting from August


    setInterval(function() {
        if (currentMonthIndex < months.length) {
            addData(myLineChart, months[currentMonthIndex], getRandomSalesValue());
            currentMonthIndex++;
        } else {
            // Optional: Reset or stop adding data once all months are covered
            // clearInterval can be used to stop the interval
            // clearInterval(intervalID);
        }
    }, 5000);

});
