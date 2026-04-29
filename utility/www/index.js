/*
Fetch data from the serial data API and return the retrieved data.

Arguments:
    numRecords: the number of records to retrieve for each metric (compensated temperature and compensated pressure) of each sensor

Return:
    the retrieved data from the JSON response
*/
async function getData(numRecords) {
    let params = new URLSearchParams();
    params.append("num_records", numRecords);
    let data = await fetch(`/api/serial_data?${params}`)
    .then(response => response.json())
    .catch(e => console.log(e));
    return data;
}

/*
Clear the dashboard.

Arguments:
    none

Return:
    nothing
*/
function clearDashboard() {
    let dashboard = document.getElementById("dashboard");
    while (dashboard.firstChild) {
        dashboard.removeChild(dashboard.firstChild);
    }
}

/*
Get the timestamps from the data, formatted to HH:MM:SS.SS.

The data object is an array containing the time-series compensated temperature and compensated pressure data for one sensor, with the following format:

[
    {
        "date_time": <date_time>,
        "compensated_temperature": <compensated_temperature>,
        "compensated_pressure": <compensated_pressure>
    },
    ...
]

Arguments:
    data: an array containing the time-series data

Return:
    an array of formatted timestamps for the data
*/
function getFormattedDateTimesFromData(data) {
    let dateTimes = data.map(x => x["date_time"].split(" ")[1].split("+")[0].slice(0, -4));
    return dateTimes;
}

/*
Create datasets from the data for use with Chart.js.

The data object is an array containing the time-series compensated temperature and compensated pressure data for one sensor, with the following format:

[
    {
        "date_time": <date_time>,
        "compensated_temperature": <compensated_temperature>,
        "compensated_pressure": <compensated_pressure>
    },
    ...
]

Each dataset mapped to in the returned object is an object with the following format:

{
    "label": <dataset_name>,
    "data": <data_values>,
    "backgroundColor": <background_color>,
    "borderColor": <line_color>,
    "tension": <tension_value>
}

Arguments:
    data: an array containing the time-series data
    datasetNames: an array containing the names of the datasets (compensated_temperature and compensated_pressure)

Return:
    an object mapping dataset names to datasets
*/
function getDatasetsFromData(data, datasetNames) {
    let lineColors = [
        "#9200b6", /* magenta */
        "#6c72e1", /* purple */
        "#33a2c1", /* blue */
        "#25be95", /* turquoise */
        "#abde57", /* green */
        "#d9664e", /* red */
    ]
    let datasets = {};
    for (let i = 0; i < datasetNames.length; i++) {
        let d = {
            "label": datasetNames[i],
            "data": data.map(x => x[datasetNames[i]]),
            "backgroundColor": "white",
            "borderColor": lineColors[i],
            "tension": 0.1,
        }
        datasets[datasetNames[i]] = d;
    }
    return datasets;
}

/*
Create a canvas element associated with a Chart.js chart object.

A dataset is used by Chart.js and has the following format:

{
    "label": <dataset_name>,
    "data": <data_values>,
    "backgroundColor": <background_color>,
    "borderColor": <line_color>,
    "tension": <tension_value>
}

Arguments:
    dateTimes: an array containing timestamps for the data
    dataset: a dataset for use with Chart.js
    units: an array containing units for each dataset

Return:
    the newly created canvas element associated with the Chart.js chart object
*/
function createGraph(dateTimes, dataset, units) {
    let graph = document.createElement("canvas");
    let chart = new Chart(graph, {
        "type": "line",
        "data": {
            "labels": dateTimes,
            "datasets": [dataset],
        },
        "options": {
            "animation": {
                "duration": 0
            },
            "elements": {
                "point": {
                    "radius": 0,
                },
            },
            "plugins": {
                "legend": {
                    "position": "top",
                }
            },
            "scales": {
                "x": {
                    "title": {
                        "display": true,
                        "text": dataset["label"],
                    }
                },
                "y": {
                    "title": {
                        "display": true,
                        "text": units,
                    }
                },
            }
        },
    });
    return graph;
}

/*
Create a dashboard element from the data.

The dashboard element will contain a graph of the data and the title of the graph. The data object is an array containing the time-series compensated temperature and compensated pressure data for one sensor, with the following format:

[
    {
        "date_time": <date_time>,
        "compensated_temperature": <compensated_temperature>,
        "compensated_pressure": <compensated_pressure>
    },
    ...
]

Arguments:
    mcuID: the microcontroller ID
    i2cPortID: the I2C port ID
    data: an array containing the time-series data

Return:
    the newly created dashboard element
*/
function createDashboardElement(mcuID, i2cPortID, data) {
    let dateTimes = getFormattedDateTimesFromData(data);
    let datasetNames = [
        "compensated_temperature",
        "compensated_pressure"
    ];
    let units = {
        "compensated_temperature": "C",
        "compensated_pressure": "mbar",
    };
    let datasets = getDatasetsFromData(data, datasetNames);
    let dashboardElement = document.createElement("div");
    dashboardElement.id = `dashboard_element_mcu_${mcuID}_i2c_port_${i2cPortID}`;
    dashboardElement.classList.add("dashboard_element");
    for (let datasetName of datasetNames) {
        let graphContainer = document.createElement("div");
        graphContainer.classList.add("graph_container");
        let title = document.createElement("p");
        title.classList.add("graph_title");
        title.innerText = `MCU ID: ${mcuID}, I2C Port: ${i2cPortID}, Statistic: ${snakeToCaps(datasetName)}`;
        let graph = createGraph(dateTimes, datasets[datasetName], units[datasetName]);
        graph.classList.add(`${datasetName}_graph`);
        graphContainer.appendChild(title);
        graphContainer.appendChild(graph);
        dashboardElement.appendChild(graphContainer);
    }
    return dashboardElement;
}

/*
Update a dashboard element with new data.

The updated dashboard element will contain a new graph with the new data appended to the existing data and the oldest data removed to keep the length the same. The data object is an array containing the time-series compensated temperature and compensated pressure data for one sensor, with the following format:

[
    {
        "date_time": <date_time>,
        "compensated_temperature": <compensated_temperature>,
        "compensated_pressure": <compensated_pressure>
    },
    ...
]

Arguments:
    dashboardElement: the canvas element associated with the Chart.js chart object to update
    data: an array containing the new time-series data to append to the existing data

Return:
    nothing
*/
function updateDashboardElement(dashboardElement, data) {
    let datasetNames = [
        "compensated_temperature",
        "compensated_pressure"
    ];
    for (let datasetName of datasetNames) {
        let graph = dashboardElement.querySelector(`.${datasetName}_graph`);
        if (graph) {
            let chart = Chart.getChart(graph);
            let updatedDateTimes = getFormattedDateTimesFromData(data);
            let updatedDatasets = getDatasetsFromData(data, datasetNames);

            // update labels
            for (let i = 0; i < updatedDateTimes.length; i++) {
                chart.data.labels.shift();
            }
            for (let i = 0; i < updatedDateTimes.length; i++) {
                chart.data.labels.push(updatedDateTimes[i]);
            }

            // update datasets
            let updatedDataset = updatedDatasets[datasetName];
            for (let i = 0; i < updatedDataset["data"].length; i++) {
                chart.data.datasets[0]["data"].shift();
            }
            for (let i = 0; i < updatedDataset["data"].length; i++) {
                chart.data.datasets[0]["data"].push(updatedDataset["data"][i]);
            }
            chart.update();
        }
    }
}

/*
Populate the dashboard with graphs of the data.

The data object must have the following structure:

{
    <mcu_0_id>: {
        <i2c_port_0>: [
            {
                "date_time": <date_time>,
                "compensated_temperature": <compensated_temperature>,
                "compensated_pressure": <compensated_pressure>
            },
            ...
        ],
        <i2c_port_1>: [
            {
                "date_time": <date_time>,
                "compensated_temperature": <compensated_temperature>,
                "compensated_pressure": <compensated_pressure>
            },
            ...
        ]
    },
    <mcu_1_id>: {
        <i2c_port_0>: [
            {
                "date_time": <date_time>,
                "compensated_temperature": <compensated_temperature>,
                "compensated_pressure": <compensated_pressure>
            },
            ...
        ],
        <i2c_port_1>: [
            {
                "date_time": <date_time>,
                "compensated_temperature": <compensated_temperature>,
                "compensated_pressure": <compensated_pressure>
            },
            ...
        ]
    }
}

<mcu_0_id>, the ID of the first microcontroller, should be 0. <mcu_1_id>, the ID of the second microcontroller (if any), should be 1. Similarly, <i2c_port_0> and <i2c_port_1> will be 0 and 1, respectively.

Arguments:
    data: an object containing the serial data for each microcontroller and each I2C port
    create: a boolean indicating whether the dashboard should be populated with new dashboard elements or whether existing elements should be updated with new data

Return:
    nothing
*/
function populateDashboard(data, create = false) {
    let dashboard = document.getElementById("dashboard");
    if (create) {
        clearDashboard();
        for (let [mcuID, mcuData] of Object.entries(data)) {
            for (let [i2cPortID, i2cPortData] of Object.entries(mcuData)) {
                let dashboardElement = createDashboardElement(mcuID, i2cPortID, i2cPortData);
                dashboard.appendChild(dashboardElement);
            }
        }
    } else {
        for (let [mcuID, mcuData] of Object.entries(data)) {
            for (let [i2cPortID, i2cPortData] of Object.entries(mcuData)) {
                let dashboardElement = document.getElementById(`dashboard_element_mcu_${mcuID}_i2c_port_${i2cPortID}`);
                updateDashboardElement(dashboardElement, i2cPortData);
            }
        }
    }
}

/*
Convert a snake-case string to a capitalized string with spaces.

Arguments:
    s: the string to convert

Return:
    the converted string
*/
function snakeToCaps(s) {
    s = s.split("_").map(x => x[0].toUpperCase().concat(x.slice(1))).join(" ");
    return s;
}

/*
Main execution function.

Arguments:
    none

Return:
    nothing
*/
function main() {
    getData(40)
    .then(data => {
        populateDashboard(data, true);
        setInterval(() => {
            getData(1)
            .then(updatedData => populateDashboard(updatedData, false));
        },
        100);
    });
}

main();
