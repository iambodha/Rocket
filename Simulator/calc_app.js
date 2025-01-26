/*
v1.2.0
(c) 2023 Janik-ux
licensed under MIT license

This program simulates the flight of a waterrocket.
*/

// Configuration presets
var data_presets = {
    // str Id: [str default val, step, str Anzeigename, bool advanced?]
    "m_Struktur": ["0.15", "0.1", "Strukturmasse (kg)", false],
    "V_T": ["1", "0.5", "Tankvolumen (l)", false],
    "d_D": ["4", "0.5", "Düsendurchmesser (mm)", false],
    "A_R": ["0.00785", "0.001", "Raketenfläche (m^2)", false],
    "p_P": ["6", "1", "Pumpendruck (bar)", false],
    "m_W_s": ["0.3", "0.1", "befüllte Wassermasse (kg)", false],
    "c_w_R": ["0.2", "0.05", "cW Rakete", false],
    "h_start": ["0", "10", "Starthöhe (m)", true]
}

var plot_presets = {
    // str Id & Anzeigename: [bool default plotten?, str yscale]
    "Height": [true, "dist"],
    "Velocity": [true, "dist"],
    "Acceleration": [true, "dist"],
    "Exhaust_Velocity": [false, "dist"],
    "Mass_Rocket": [false, "mass"],
    "Inner_Air_Mass": [false, "mass"],
    "Inner_Water_Mass": [false, "mass"],
    "Inner_Air_Pressure": [false, "pres"],
}

var flight_data
var xlabel = "Time"
var outsidesgraph = null

// Core calculation functions
function calc() {
    // "naturkonstanten" und Umwelteinflüsse
    var R = 8.3145; // J/(mol*kg)
    var M = 0.028949; // kg/mol
    var G = 6.67259 * Math.pow(10, -11); // Gravitationskonstante
    var m_Erde = 5.972 * Math.pow(10, 24); // kg
    var radius_Erde = 6.371000785 * Math.pow(10, 6); // m
    var rho_W = 997; // kg/m^3
    var p_A0 = 1.01325 * Math.pow(10, 5); // Pa Außendruck bei 0m ü NN nach ISA
    var T = 288.15; // K Temp bei 0m nach ISA
    var h_Boden = 100; // m ü NN

    // variable Umwelteinflüsse
    function p_A(h) {
        return p_A0 * (1 - 0.0065 * h / T); // Pa
    }
    function rho_L_A(h) {
        var p;
        p = p_A(h);
        return p * M / (R * T); // kg/m^3
    }
    function g(h) {
        return -(G * m_Erde) / Math.pow(h + radius_Erde, 2); // m/s^2
    }

    // Raketenspezifikationen
    var V_T = document.getElementById("V_T").valueAsNumber / 1000; // L to 
    var m_W_s = document.getElementById("m_W_s").valueAsNumber; // kg
    var m_Struktur = document.getElementById("m_Struktur").valueAsNumber; // kg
    var m_Nutz = 0; // kg
    var p_P = document.getElementById("p_P").valueAsNumber * Math.pow(10, 5); // Pa
    var A_R = document.getElementById("A_R").valueAsNumber; // m^2
    var d_D = document.getElementById("d_D").valueAsNumber; // mm
    var c_w_R = document.getElementById("c_w_R").valueAsNumber;
    var h_start = document.getElementById("h_start").valueAsNumber; // m über h_Boden

    // Einstellungen
    var dt = 0.001;
    var max_t = 20;
    var debug = false;
    var calculate_LW = true;

    // vom Programm berechnete Werte
    var p_0 = p_A(h_start+h_Boden) + p_P; // Pa
    var V_W_s = m_W_s / rho_W; // m^3
    var m_L_s = (V_T - V_W_s) * (p_0 * M / (R * T)); // kg
    var A_D = 1 / 4 * Math.PI * Math.pow(d_D / 1000, 2); // m^2
    var m_R_s = m_Struktur + m_Nutz + m_L_s + m_W_s; // m

    // initialisieren der Speicherstrukturen für die Werte
    var v_str_list = [];
    var t_list = [];
    var V_W_list = [];
    var m_W_list = [];
    var p_L_list = [];
    var m_L_list = [];
    var h_R_list = [];
    var v_R_list = [];
    var a_R_list = [];
    var m_R_list = [];
    var a_R_Luft_list = [];
    var twr_list = [];

    // Über die Berechnung variable Werte
    var h_R = h_start; // Höhe über dem Boden
    var v_R = 0;
    var m_R = m_R_s;
    var V_W = V_W_s;
    var m_L = m_L_s;
    var t = 0;

    while (t < max_t && h_R >= 0) {
        let rho_L = m_L / (V_T - V_W);
        let p_L = Math.round(rho_L * R * T / M, 10);

        if (p_L < p_A(h_R+h_Boden)) {
            p_L = p_A(h_R+h_Boden);
            rho_L = p_L * M / (R * T);
            m_L = rho_L * V_T;
        }

        if (calculate_LW) {
            let richtung = v_R > 0 ? -1 : 1;
            var F_Luft = 1 / 2 * rho_L_A(h_R+h_Boden) * A_R * c_w_R * Math.pow(v_R, 2) * richtung;
            var dv_R_Luft = F_Luft / m_R * dt;
        } else {
            var dv_R_Luft = 0;
        }

        if (V_W > 0) {
            var v_str = Math.pow(2 * (p_L - p_A(h_R+h_Boden)) / rho_W, 0.5);
            var dV_W = v_str * A_D * dt;
            var F_R_str = dV_W * rho_W * v_str;
            var dv_R_str = F_R_str / m_R;
            var dm_L = 0;
        } else {
            var v_str = Math.pow(2 * (p_L - p_A(h_R+h_Boden)) / rho_L, 0.5);
            var dm_L = v_str * A_D * dt * rho_L;
            var F_R_str = dm_L * v_str;
            var dv_R_str = F_R_str / m_R;
            var dV_W = 0;
        }

        var twr = Math.abs(F_R_str) / Math.abs(g(h_R+h_Boden) * dt * m_R);
        var dv_R = dv_R_str + g(h_R+h_Boden) * dt + dv_R_Luft;
        v_R += dv_R;
        h_R += v_R * dt;

        if (debug) {
            console.log(`V_W: ${V_W}`);
            console.log(`m_L: ${m_L}`);
            console.log(`v_str: ${v_str}`);
            console.log(`dV: ${v_str * A_D * dt}`);
            console.log(`rho_L: ${rho_L}`);
            console.log(`p_L: ${p_L}`);
        }

        v_str_list.push(v_str);
        t_list.push(t.toFixed(((dt.toString()).split(".")[1]).length));
        V_W_list.push(V_W);
        p_L_list.push(p_L);
        m_L_list.push(m_L);
        m_W_list.push(V_W*rho_W)
        v_R_list.push(v_R);
        a_R_list.push((dv_R / dt)/9.81);
        h_R_list.push(h_R);
        m_R_list.push(m_R);
        a_R_Luft_list.push(dv_R_Luft / dt);
        twr_list.push(twr);
        V_W -= dV_W;

        if (V_W < 0) {
            dV_W += V_W;
            V_W = 0;
        }

        m_R -= dV_W * rho_W;
        m_R -= dm_L;
        m_L -= dm_L;
        t += dt;
    }

    console.log(`Total Iterations: ${Math.round(t / dt, 0)}`);
    console.log("Max Height:", Math.max(...h_R_list))
    return {
        "Time": t_list,
        "Height": h_R_list,
        "Velocity": v_R_list,
        "Acceleration": a_R_list,
        "Exhaust_Velocity": v_str_list,
        "Mass_Rocket": m_R_list,
        "Inner_Air_Mass": m_L_list,
        "Inner_Water_Mass": m_W_list,
        "Inner_Air_Pressure": p_L_list
    }
}

// UI and display functions
function plot() {
    if (outsidesgraph != null) {
        outsidesgraph.destroy();
        outsidesgraph = null;
    }
    var canvas = document.getElementById("graph")
    var ctx_out = canvas.getContext('2d')

    // select data to plot
    var data = {}
    var yscales = [] // which kinds/scale of data will be plot (mass, dist, ...)
    for (const [key, value] of Object.entries(plot_presets)) {
        if (document.getElementById(key).checked == true) {
            data[key] = flight_data[key]
            yscales.indexOf(value[1]) === -1 && yscales.push(value[1])
        }
    }

    // check if there is no data to plot
    if (Object.keys(data).length == 0) {
        console.log("NOTHING TO PLOT!")
        canvas.style.width = '100%';
        ctx_out.font = "20px Arial"
        ctx_out.textAlign = "center"
        ctx_out.fillText("Nothing to plot :(", canvas.width/2, canvas.height/2)
        return
    }

    // select which data to plot at label at x axis
    var labels = flight_data[xlabel]

    // set up axes config
    var AxesConf = {
        x: {
            title: {
                display: true,
                text: xlabel
            },
        }};

    // dynamically add y axes to the sides for different units
    var wasr = true;
    for (scaletype of yscales) {
        AxesConf[scaletype] = {
            type: 'linear',
            position: (wasr ? 'left' : 'right'),
        }
        wasr = !wasr
    }
    console.log(AxesConf)

    // prepare plotting dataset
    var colorarr = ["red", "green", "black", "blue", "grey", "violet"]
    var dataset = []
    for (const [key, value] of Object.entries(data)) {
        dataset.push(
            {
                label: key.replace(/_/g, " "),
                yAxisID: plot_presets[key][1], 
                data: value,
                fill: false,
                borderColor: colorarr[0],
                borderWidth: 0.75
            }
        )
        colorarr.push(colorarr[0])
        colorarr.splice(0, 1)
    }

    // plot new graph
    outsidesgraph = new Chart(ctx_out, {
        type: 'line',
        options:{
            responsive: true,
            maintainAspectRatio: false,
            interaction: {
                mode: 'index',
                intersect: false
            },
            scales: AxesConf,
            elements: {
                point:{
                    radius: 0
                }
            }
        },
        data: {
            labels: labels,
            datasets: dataset,
        }
    });
    jump("graphwrapper")
}

// Initialization and control functions
function init() {
    // add inputs for the user data
    for (const [key, value] of Object.entries(data_presets)) {
        html = `<div class="inputfield">
                    <label class="inputname">${value[2]}</label>
                    <input id="${key}" class="listinput" type="number" value=${value[0]} step=${value[1]}>
                </div>`
        if (value[3] == true) {
            document.getElementById("adv_data_inp_header").insertAdjacentHTML("afterend", html);
        }
        else {
            document.getElementById("data_inp_header").insertAdjacentHTML("afterend", html);
        }
    }

    // add buttons which data to plot
    for (const [key, value] of Object.entries(plot_presets)) {
        html = `<label class="whichshow">
                    <input type="checkbox" onclick=plot() ${value[0] == true ? "checked" : ""} id=${key}>
                    <span class="whichshow_checkmark">${key.replace(/_/g, " ")}</span>
                </label>`
        document.getElementById("plot_inp_header").insertAdjacentHTML("afterend", html);
    }

    // compute the values to plot
    run()
}

function run() {
    flight_data = calc()
    plot()
}

function reset() {
    for (const [key, value] of Object.entries(data_presets)) {
        document.getElementById(key).value = value[0];
    }
    for (const [key, value] of Object.entries(plot_presets)) {
        document.getElementById(key).checked = value[0];
    }
    run()
}

// Navigation helper functions
function jump(h) {
    var url = location.href;
    location.href = "#"+h;
    history.replaceState(null,null,url);
}

function fscreen(btn, element_id) {
    // make full screen
    var elem = document.getElementById(element_id);
    elem.classList.remove("graphwrapper_norm");
    elem.classList.add("graphwrapper_max");
    btn.setAttribute("style", "top: 0px"); // move btn into the field to not exclude it from screen
    jump(element_id);

    // change button to "leave"
    btn.setAttribute("onclick", "leave_fscreen(this, '" + element_id + "');");
    btn.innerHTML = "&#10005;";
}

function leave_fscreen(btn, element_id) {
    // make full screen
    var elem = document.getElementById(element_id);
    elem.classList.remove("graphwrapper_max");
    elem.classList.add("graphwrapper_norm");
    btn.setAttribute("style", "top: -15px"); // move btn again up a little to not stack it into graph
    jump("");

    // change button to "fullscreen"
    btn.setAttribute("onclick", "fscreen(this, '" + element_id + "');");
    btn.innerHTML = "&#x26F6;";
}

// Add this at the beginning of your JavaScript file
document.getElementById('theme-toggle').addEventListener('change', function(event) {
    if (event.target.checked) {
        document.documentElement.setAttribute('data-theme', 'dark');
        localStorage.setItem('theme', 'dark');
    } else {
        document.documentElement.setAttribute('data-theme', 'light');
        localStorage.setItem('theme', 'light');
    }
});

// Check for saved theme preference
const savedTheme = localStorage.getItem('theme') || 'light';
document.documentElement.setAttribute('data-theme', savedTheme);
document.getElementById('theme-toggle').checked = savedTheme === 'dark';

// Add this to your existing JavaScript file

function downloadCSV(data, filename) {
    // Convert data to CSV string
    let csvContent = "data:text/csv;charset=utf-8,";
    
    // Get all available data types
    const dataTypes = Object.keys(data);
    
    // Add header row
    csvContent += "Time," + dataTypes.filter(type => type !== "Time").join(",") + "\n";
    
    // Add data rows
    for (let i = 0; i < data.Time.length; i++) {
        let row = [data.Time[i]];
        dataTypes.forEach(type => {
            if (type !== "Time") {
                row.push(data[type][i]);
            }
        });
        csvContent += row.join(",") + "\n";
    }
    
    // Create download link
    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", filename);
    document.body.appendChild(link);
    
    // Trigger download
    link.click();
    document.body.removeChild(link);
}

function exportSelectedData() {
    const exportSelect = document.getElementById('export-select');
    const selectedOption = exportSelect.value;
    let dataToExport = { Time: flight_data.Time };
    
    if (selectedOption === 'all') {
        // Export all data
        dataToExport = flight_data;
    } else if (selectedOption === 'visible') {
        // Export only visible graph data
        Object.keys(plot_presets).forEach(key => {
            if (document.getElementById(key).checked) {
                dataToExport[key] = flight_data[key];
            }
        });
    }
    
    const timestamp = new Date().toISOString().slice(0,19).replace(/[:]/g, '-');
    downloadCSV(dataToExport, `rocket-data-${timestamp}.csv`);
}
