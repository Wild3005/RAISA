// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

console.log("halo");

// Connect to the ROS bridge WebSocket server
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

ros.on("connection", function () {
    console.log("Connected to WebSocket server.");
});

ros.on("error", function (error) {
    console.log("Error connecting to WebSocket server:", error);
});

ros.on("close", function () {
    console.log("Connection to WebSocket server closed.");
});

// Create a ROSLIB Topic to subscribe to the 'ui_test' topic
var listener = new ROSLIB.Topic({
    ros: ros,
    name: "/ui_test",
    messageType: "std_msgs/String",
});

listener.subscribe(function (message) {
    console.log("Received message:", message.data);
});



// ============================================================
// Publisher untuk /button/mode
// ============================================================
const buttonModePub = new ROSLIB.Topic({
    ros: ros,
    name: '/button/mode',
    messageType: 'std_msgs/Int8'
});

// ============================================================
// Button Event Listeners: Mode Selection
// ============================================================

// Mode Interaksi (10)
document.getElementById('btn-interaction').addEventListener('click', function() {
    const msg = new ROSLIB.Message({
        data: 10  // MODE_INTERACTION = 10
    });
    buttonModePub.publish(msg);
    console.log('Published MODE_INTERACTION (10) to /button/mode');
    this.classList.add('is-loading');
    setTimeout(() => this.classList.remove('is-loading'), 500);
});

// Mode Navigasi (11)
document.getElementById('btn-navigation').addEventListener('click', function() {
    const msg = new ROSLIB.Message({
        data: 11  // MODE_NAVIGATION = 11
    });
    buttonModePub.publish(msg);
    console.log('Published MODE_NAVIGATION (11) to /button/mode');
    this.classList.add('is-loading');
    setTimeout(() => this.classList.remove('is-loading'), 500);
});

// ============================================================
// Publisher untuk /button/case_state
// ============================================================
var button_control_pub = new ROSLIB.Topic({
    ros: ros,
    name: "/button/case_state",
    messageType: "std_msgs/Int8", // ROS2 TYPE
});

var keyboard_control_pub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui_keyboard_control",
    messageType: "std_msgs/String", // ROS2 TYPE
});
    console.log("Generating button ");

// ============================
// BUTTON GENERATOR (12 BUTTONS)
// ============================
const btnGroup = document.getElementById("btn-group");

for (let i = 1; i <= 12; i++) {
    console.log("Generating button " + i);
    const btn = document.createElement("button");
    btn.className = "button is-primary m-2";
    if( i === 0){
        btn.innerText = "SIT";
    }else if( i === 1){
        btn.innerText = "STAND";
    }else{
        btn.innerText = "BTN " + i;
    }

    btn.onclick = () => sendCommandButton(i);
    btn.tabIndex = -1;   // ✅ cegah fokus nempel

    btnGroup.appendChild(btn);
}

// ============================
// SEND DATA TO ROS2
// ============================
function sendCommandButton(btnId) {
    // Contoh payload: [button_id, timestamp_mod_127]
    // send Int8 data
    const payload = new ROSLIB.Message({
        data: btnId,
    });

    button_control_pub.publish(payload);

    console.log("📤 Sent:", payload.data);
}