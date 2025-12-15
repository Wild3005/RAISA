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

// Mode Button Topic - WRAP DALAM DOMContentLoaded
document.addEventListener('DOMContentLoaded', function() {
    let modeTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/button/mode',
        messageType: 'std_msgs/Int8'
    });

    const btnInteraction = document.getElementById('btn-interaction');
    const btnNavigation = document.getElementById('btn-navigation');

    if (btnInteraction) {
        btnInteraction.addEventListener('click', () => {
            let msg = new ROSLIB.Message({ data: 0 }); // 0 = Interaksi
            modeTopic.publish(msg);
            console.log('Mode: Interaksi');
        });
    } else {
        console.error('btn-interaction not found!');
    }

    if (btnNavigation) {
        btnNavigation.addEventListener('click', () => {
            let msg = new ROSLIB.Message({ data: 1 }); // 1 = Navigasi
            modeTopic.publish(msg);
            console.log('Mode: Navigasi');
        });
    } else {
        console.error('btn-navigation not found!');
    }
});