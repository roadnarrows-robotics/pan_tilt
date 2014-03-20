// 
// Pan-tilt control rosbridge handle
// 
// required js imports:
//    http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js
//    http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js
//    rosbridge_global.js
//

function panTilt(throttle_rate) {
  // set default throttle rate = 0
  this.throttle_rate = typeof throttle_rate !== 'undefined' ? throttle_rate : 0;
  
  this.sub_state = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
   }
    this.robot_status.subscribe(function(msg){cb(msg);});
  }


  this.calibrate = function(cb) {
    cb= typeof cb !== 'undefined' ? cb : function(rsp){};
    var msg = {force_recalib:1};
    var goal = new ROSLIB.Goal({ 
      actionClient: this.calibrate_client,
      goalMessage: msg
    });
    return goal;
  }

  this.sub_feedback = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.calibrate_feedback.subscribe(function(msg){cb(msg);});
  }
  
  this.sub_robotStatus = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.robot_status_EX.subscribe(function(msg){cb(msg);});
  }
  
  this.sub_jointStates = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.joint_states_EX.subscribe(function(msg){cb(msg);});
  }

  this.sub_jointTrajectory = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.joint_trajectory.subscribe(function(msg){cb(msg);});
  }
  
  this.EStop = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.EStop_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.resetEStop = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.resetEStop_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.freeze = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.freeze_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.release = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.release_srv.callService(req, function(rsp){cb(rsp);});
  }
      
  this.clearAlarms = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.clearAlarms_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.ProductInfo = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.getProductInfo_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.zeroPoint = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.goToZeroPt_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.pan = function(r, cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest(r);
    this.pan_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.sweep = function(r, cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest(r);
    this.sweep_srv.callService(req, function(rsp){cb(rsp);});
  }
  
  this.getLoggers = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.getLoggers_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.draw = function ()
  {
   create_panel();
   Table();
   create_buttons();
  } 

  //------------------------------------------------------------------------//
  //                          PRIVATE IMPLEMENTATION                        //
  //------------------------------------------------------------------------//

  //Action Servers
  this.calibrate_client = new ROSLIB.ActionClient({
    ros: ros,
    serverName: "/pan_tilt_control/calibrate_as",
    actionName: "pan_tilt_control/CalibrateAction"
  });

  //Topics
  this.robot_status = new ROSLIB.Topic({
    ros: ros,
    name: "/pan_tilt_control/robot_status",
    messageType: "industrial_msgs/RobotStatus",
    throttle_rate : this.throttle_rate 
  });

  this.calibrate_feedback = new ROSLIB.Topic({
    ros: ros,
    name: "/pan_tilt_control/calibrate_as/feedback",
    messageType: "pan_tilt_control/CalibrateActionFeedback"
  });
 
  this.joint_trajectory = new ROSLIB.Topic({
    ros: ros,
    name: "/pan_tilt_control/joint_command",
    messageType: "trajectory_msgs/JointTrajectory"
  });
  
  this.robot_status_EX = new ROSLIB.Topic({
    ros:ros,
    name: "/pan_tilt_control/robot_status_ex",
    messageType: "pan_tilt_control/RobotStatusExtended"
  });
  
  this.joint_states_EX = new ROSLIB.Topic({
    ros:ros,
    name: "/pan_tilt_control/joint_states_ex",
    messageType: "pan_tilt_control/JointStateExtended"
  });

  //Services
  this.getProductInfo_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/get_product_info",
    messageType: "/pan_tilt_control/ProductInfo"
  });

  this.EStop_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/estop",
    messageType: "/pan_tilt_control/Estop"
  });

  this.pan_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/pan",
    messageType: "/pan_tilt_control/Pan"
  });
  
  this.setRobotMode_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/set_robot_mode",
    messageType: "/pan_tilt_control/SetRobotMode"
  });

  this.setLoggerLevel_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/set_logger_level",
    messageType: "/pan_tilt_control/SetLoggerLevel"
  });

  this.isAlarmed_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/is_alarmed",
    messageType: "/pan_tilt_control/IsAlarmed"
  });

  this.resetEStop_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/reset_estop",
    messageType: "/pan_tilt_control/ResetEStop"
  });

  this.freeze_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/freeze",
    messageType: "/pan_tilt_control/Freeze"
  });
 
  this.stop_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/stop",
    messageType: "/pan_tilt_control/Stop"
  });
 
  this.clearAlarms_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/clear_alarms",
    messageType: "/pan_tilt_control/ClearAlarms"
  });

  this.release_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/release",
    messageType: "/pan_tilt_control/Release"
  });

  this.goToZeroPt_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/goto_zero",
    messageType: "/pan_tilt_control/GotoZeroPt"
  });
 
  this.getLoggers_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/get_loggers",
    messageType: "/pan_tilt_control/GetLoggers"
  });
  
  this.sweep_srv = new ROSLIB.Service({
    ros: ros,
    name: "/pan_tilt_control/sweep",
    messageType: "/pan_tilt_control/Sweep"
  });
}
pan = new panTilt(50);
