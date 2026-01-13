// Navbar Animation
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    backgroundColor: "#fff",
    duration: 2000,
    loop: true,
});

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

// ============================
// NAVBAR ANIMATION
// ============================
anime({
    targets: ".navbar-svgs path",
    strokeDashoffset: [anime.setDashoffset, 0],
    easing: "easeInOutExpo",
    duration: 2000,
    loop: true,
});

// ============================
// ROS CONNECTION
// ============================
var ros = new ROSLIB.Ros({
    url: "ws://" + window.location.hostname + ":9090",
});

ros.on("connection", () => console.log("✅ Connected to ROSBridge"));
ros.on("error", (e) => console.log("❌ ROS Error:", e));
ros.on("close", () => console.log("⚠️ ROS Connection Closed"));

// ============================
// ROS2 PUBLISHER (Int8)
// ============================
var button_control_pub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui_control",
    messageType: "std_msgs/Int8", // ROS2 TYPE
});

var keyboard_control_pub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui_keyboard_control",
    messageType: "std_msgs/String", // ROS2 TYPE
});

// ============================
// KEYBOARD LISTENER
// ============================
window.addEventListener("keydown", function (event) {
    let command = null;

    command = event.key;

    if (command) {
        sendKeyboardCommand(command);
    }
});

window.addEventListener("keyup", function (event) {
    let command = "stop";
    sendKeyboardCommand(command);
});

window.addEventListener("keydown", function (event) {
    event.preventDefault();
    sendKeyboardCommand(event.key);
});


function sendKeyboardCommand(command) {
    const payload = new ROSLIB.Message({
        data: command,
    });

    keyboard_control_pub.publish(payload);

    console.log("📤 Sent:", payload.data);
}   


// ============================
// BUTTON GENERATOR (12 BUTTONS)
// ============================
const btnGroup = document.getElementById("btn-group");

for (let i = 1; i <= 12; i++) {
    const btn = document.createElement("button");
    btn.className = "button is-primary m-2";
    btn.innerText = "BTN " + i;

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

const stdout1 = document.getElementById("stdout1");
stdout1.src =
  "http://" +
  window.location.hostname +
  ":8080/stream?topic=" +
  "/vision/pose_frame" +
  "&quality=10";
stdout1.alt = "MJPEG Stream";

function updateFloat(id, value) {
    document.getElementById(id).innerText = value.toFixed(2);
}

function updateInt(id, value) {
    document.getElementById(id).innerText = value;
}

var robotPoseSub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui/robot/pose2d",
    messageType: "geometry_msgs/Pose2D"
});

robotPoseSub.subscribe(msg => {
    updateFloat("robot_x", msg.x);
    updateFloat("robot_y", msg.y);
    updateFloat("robot_th", msg.theta);

    console.log(msg);
});

var robotModeSub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui/robot/mode",
    messageType: "std_msgs/Int8"
});

robotModeSub.subscribe(msg => {
    updateInt("robot_nav_mode", msg.data);
});


var robotFSMSub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui/robot/fsm_mode",
    messageType: "std_msgs/Int8"
});

robotFSMSub.subscribe(msg => {
    updateInt("robot_fsm_mode", msg.data);
});
var robotFollowSub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui/robot/following_mode",
    messageType: "std_msgs/Int8"
});

robotFollowSub.subscribe(msg => {
    updateInt("robot_follow_mode", msg.data);
});
var humanPoseSub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui/human/pose2d",
    messageType: "geometry_msgs/Pose2D"
});

humanPoseSub.subscribe(msg => {
    updateFloat("human_x", msg.x);
    updateFloat("human_y", msg.y);
    updateFloat("human_th", msg.theta);
});
var humanVelSub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui/human/velocity",
    messageType: "geometry_msgs/Twist"
});

humanVelSub.subscribe(msg => {
    updateFloat("human_vx", msg.linear.x);
    updateFloat("human_vy", msg.linear.y);
    updateFloat("human_vth", msg.angular.z);
});
var humanModeSub = new ROSLIB.Topic({
    ros: ros,
    name: "/ui/human/mode",
    messageType: "std_msgs/Int8"
});

humanModeSub.subscribe(msg => {
    updateInt("human_mode", msg.data);
});

