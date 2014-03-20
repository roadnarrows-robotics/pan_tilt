window.setInterval(function ()
          {
            updateData();
          },500)

var calibrate_action_goal;
var robotState;
var jointState;
var i=0;

function buttons_panel(id, name, side, color)
{
  d3.select("#" + side + "Column").append("button")
      .attr("id", id)
      .attr("class", side)
      .style("background-color", color)
      .style("background-image", "url('img/" + id + ".png')" )
      .style("text-align", "right")
      .text(name)
      .attr("onclick", id + "()");

}

var first;
var second;
var third;
var fourth;
var fifth;
var sixth;

function create_panel() {
d3.select("#leftColumn").append("imageleft")
    .style("background-image", "url(img/RoadNarrows.png)");

d3.select("#rightColumn").append("imageright")
    .style("background-image", "url(img/Logo.png)");

buttons_panel("Calibrate", "Calibrate", "left", "#ccc");
buttons_panel("ZeroPoint", "Zero Point", "left", "#ccc");
buttons_panel("Pan", "Pan", "left", "#ccc");
buttons_panel("Sweep", "Sweep", "left", "#ccc");
buttons_panel("SpecifyMove", "Specify Move", "left", "#ccc");
buttons_panel("EStop", "EStop", "right", "#900");
buttons_panel("Freeze", "Freeze", "right", "#ccc");
buttons_panel("Release", "Release", "right", "#ccc");
buttons_panel("ClearAlarms", "Clear Alarms", "right", "#ccc");
buttons_panel("Settings", "Settings", "right", "#ccc");
buttons_panel("About", "About", "right", "#ccc");
buttons_panel("Quit", "Quit", "right", "#ccc");

d3.select("#centerColumn").append("pantilt")
    .text("Pan-Tilt Control Panel");
d3.select("#centerColumn").append("title");
d3.select("#centerColumn").append("main")
    .text("Joint State");
d3.select("#centerColumn").append("box")
    .text("NA");

first  = d3.select("title").append("sbar")
    .text("Mode:");

second = d3.select("title").append("sbar")
    .text("State:");

third  = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("Motors");

fourth = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("Moving");

fifth  = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("Alarms");

sixth  = d3.select("title").append("sbar")
    .style("background-image", "url('img/dark.png')")
    .text("EStop");
}

function create_buttons()
{
  d3.select("main").append("topbutton").style("background-image", "url('img/Up.png')").text("Tilt Up").attr("onclick", "Up()");
  d3.select("main").append("tablebutton").style("background-image", "url('img/Left.png')").text("Pan Left").attr("onclick", "Left()");

  d3.select("main").append("tablebutton").style("background-image", "url('img/Down.png')").text("Tilt Down").attr("onclick", "Down()");

  d3.select("main").append("tablebutton").style("background-image", "url('img/Right.png')").text("Pan Right").attr("onclick", "Right()");

  //Initilizing bottom status bar
 d3.select("box").text("Pan-Tilt interface initialized."); 
}

function EStop()
{
  console.info("EStop");
 var r=confirm("Warning: All motors will become undriven. The pan-tilt may fall and be damaged.");
    if (r==false)
    { 
      d3.select("#resetEStop")
          .text("EStop")
          .style("background-color","#900")
          .attr("id","EStop")
          .attr("onclick","EStop()");
    }
    else
    {
      pan.EStop();
      d3.select("box").text("Pan-Tilt emergency stopped.");
      d3.select("#EStop")
          .text("Reset EStop")
          .style("background-color", "#090")
          .attr("id","resetEStop")
          .attr("onclick","resetEStop()");
    }
} 

function resetEStop()
{
  console.info("EStop Reset");
  d3.select("box").text("Pan-Tilt emergency stop has been reset.");
  d3.select("#resetEStop")
      .text("EStop")
      .style("background-color","#900")
      .attr("id","EStop")
      .attr("onclick","EStop()");
  pan.resetEStop();
}

function Freeze()
{
  console.info("Freeze");
  d3.select("box").text("Pan-Tilt has been frozen at current position.");
  pan.freeze();
}

function Release()
{
  console.info("Release");
  var r=confirm("Warning: All motors will become undriven. The pan-tilt may fall and become damaged.");
  if (r==false)
  {
    console.info("Did not release arm")
  }
  else
  {
    d3.select("box").text("Pan-Tilt has been released, all motors are unpowered.");
    pan.release();
  }
}

function Calibrate()
{
  console.info("Calibrate")
  var r=confirm("Warning: the pan-tilt should be placed near the Zero point and the workspace cleared of obstructions");
  if (r==false)
  {
    d3.select("#cancelCalibrate")
        .text("Calibrate")
        .style("background-color","#ccc")
        .style("background-image", "url('img/Calibrate.png')")
        .attr("id","Calibrate")
        .attr("onclick","Calibrate()");
  }
  else
  {
    d3.select("box").text("Calibrating pan-tilt.");
    calibrate_action_goal = pan.calibrate();
    calibrate_action_goal.send();
    d3.select("#Calibrate")
        .text("Cancel Calibrate")
        .style("background-color", "#900")
        .style("background-image", "url('img/cancelCalibrate.png')")
        .attr("id","cancelCalibrate")
        .attr("onclick","cancelCalibrate()");
    pan.sub_feedback(function(message) {
    d3.select("box").text("Calibrating complete.");
    }) 
  }  
}

function cancelCalibrate()
{
  var r=confirm("Warning: The pan-tilt MUST be calibrated before operation. Calibration will have to be restarted to use the pan-tilt");
  if (r==false)
  {
    calibrate_action_goal = pan.calibrate();
    calibrate_action_goal.send();
    d3.select("#Calibrate")
        .text("Cancel Calibrate")
        .style("background-color", "#900")
        .style("background-image", "url('img/cancelCalibrate.png')")
        .attr("id","cancelCalibrate")
        .attr("onclick","cancelCalibrate()"); 
  }
  else
  {
    console.info("STOP calibration")
    calibrate_action_goal.cancel();
    d3.select("#cancelCalibrate")
        .text("Calibrate")
        .style("background-color","#ccc")
        .style("background-image", "url('img/Calibrate.png')")
        .attr("id","Calibrate")
        .attr("onclick","Calibrate()");
    d3.select("box").text("Calibrating canceled.");
  }
}

function ZeroPoint()
{
  console.info("Zero Point");
  pan.zeroPoint();
  d3.select("box").text("Zero Point");
}

//Default Values for Pan Service
var MINPAN_rad; 
var MAXPAN_rad; 

var VELPAN     = 20;        //Pan velocity
var MINPAN_deg = -125.0;    //Degrees Minimum Pan position
var MAXPAN_deg = 125.0;     //Degrees Maximum Pan position

function Pan()
{
  console.info("Pan");
  MINPAN_rad = (MINPAN_deg * Math.PI) / 180;
  MAXPAN_rad = (MAXPAN_deg * Math.PI) / 180;
  var req = {"min_pos": MINPAN_rad, "max_pos": MAXPAN_rad, "velocity": VELPAN}
  d3.select("box").text("Panning Continuously");
  pan.pan(req);
}

//Default Values for Sweep Service
var SW_MINPAN_rad;  
var SW_MAXPAN_rad;  
var SW_MINTILT_rad; 
var SW_MAXTILT_rad; 

var SW_VELPAN      = 20;     //Sweep Pan Velocity
var SW_VELTILT     = 20.0;   //Sweep Tilt Velocity
var SW_MINPAN_deg  = -125.0; //Degrees Sweep Minimum Pan Position
var SW_MAXPAN_deg  = 125.0;  //Degrees Sweep Maximum Pan Position
var SW_VELPAN      = 20;     //Sweep Pan Velocity
var SW_MINTILT_deg = 10.0;   //Degrees Sweep Minimum Tilt Position
var SW_MAXTILT_deg = 90.0;   //Degrees Sweep Maximum Tilt Position
var SW_VELTILT     = 20.0;   //Sweep Tilt Velocity

function Sweep()
{
  console.info("Sweep");
  SW_MINPAN_rad = (SW_MINPAN_deg * Math.PI) / 180;
  SW_MAXPAN_rad = (SW_MAXPAN_deg * Math.PI) / 180;
  SW_MINTILT_rad = (SW_MINTILT_deg * Math.PI) / 180;
  SW_MAXTILT_rad = (SW_MAXTILT_deg * Math.PI) / 180;
  var req = {"pan_min_pos": SW_MINPAN_rad, "pan_max_pos": SW_MAXPAN_rad, "pan_velocity": SW_VELPAN, "tilt_min_pos": SW_MINTILT_rad, "tilt_max_pos": SW_MAXTILT_rad, "tilt_velocity": SW_VELTILT}
  d3.select("box").text("Sweeping Continuously");
  pan.sweep(req);
}

function Settings()
{
  console.info("Settings");
  settingOverlay();
}

function SpecifyMove()
{
  console.info("Setting Position");
  specifyMoveOverlay();
}

function specifyMoveOverlay()
{
  el = document.getElementById("overlay");
  el.style.visibility = (el.style.visibility == "visible")? "hidden":"visible";
  var foo = d3.select("#overlay").selectAll("p");
  foo.style("background-image", null)
      .html("<h2>Specify Trajectory</h2>")
      .append("block")
      .html("<b>tilt</b>" + "<br/>"  
            + "Position(degrees):<input type=text id=tiltPos size=5px value=10.0>" + "<br/>"
            + "Velocity(%max):<input type=text id=tiltVel size=5px value=10.0>" + "<br/> <br/>"
            +"<b>pan</b>" + "<br/>"
            + "Position(degrees):<input type=text id=PanPos size=5px value=10.0>" + "<br/>"
            + "Velocity(%max):<input type=text id=PanVel size=5px value=10.0>" + "<br/>"); 
  foo.append("savebutton")
      .text("Cancel")
      .attr("onclick","specifyMoveOverlay()");
  foo.append("savebutton")
      .text("Move")
      .attr("onclick","move()");
}

function move()
{
  console.info("Moving");
  d3.select("box").text("Move on specified trajectory.");
  specifyMoveOverlay();
  //add service here 
}

function settingOverlay()
{
  el = document.getElementById("overlay");
  el.style.visibility = (el.style.visibility == "visible")? "hidden":"visible";
  var foo = d3.select("#overlay").selectAll("p")
  foo.style("background-image", null)
      .html(
        "<form><input type=checkbox id=check checked/>show warning dialog before calibrating pan-tilt.</form>" 
      + "<form><input type=checkbox id=again checked/>Force (re)calibration for all joints on calibrate action.</form>" 
      + "<form><input type=checkbox id=last checked/>Show warning dialog before releasing pan-tilt.</form>")  
      .append("block")
      .html("<b>Pan Parameters</b>" + "<br/>" 
            + "Pan Minimum Position(degrees):<input type=text id=minPan size=5px value=" + MINPAN_deg +  ">" + "<br/>" 
            + "Pan Maximum Position(degrees):<input type=text id=maxPan size=5px value=" + MAXPAN_deg + ">" + "<br/>" 
            + "Pan Velocity(%):<input type=text id=velPan size=5px value=" + VELPAN + ">" + "<br/>")
      .append("block")
      .html("<b>Sweep Parameters</b>" + "<br/>"
            + "Pan Minimum Position (degrees):<input type=text id=sw_minPan size=5px value=" + SW_MINPAN_deg + ">" + "<br/>"
            + "Pan Maximum Position (degrees):<input type=text id=sw_maxPan size=5px value=" + SW_MAXPAN_deg + ">" + "<br/>" 
            + "Pan Velocity(%):<input type=text id=sw_velPan size=5px value=" + SW_VELPAN + ">" + "<br/>"
            + "Tilt Minimum Position (degrees):<input type=text id=sw_minTilt size=5px value=" + SW_MINTILT_deg + ">" + "<br/>"
            + "Tilt Maximum Position (degrees):<input type=text id=sw_maxTilt size=5px value=" + SW_MAXTILT_deg + ">" + "<br/>"
            + "Tilt Velocity (%):<input type=text id=sw_velTilt size=5px value=" + SW_VELTILT + ">" + "<br/>")
  foo.append("savebutton")
      .text("Cancel")
      .attr("onclick","settingOverlay()");
  foo.append("savebutton")
      .text("OK")
      .attr("onclick","save()");
}

function save()
{
  MINPAN_deg     = Number(document.getElementById("minPan").value);
  MAXPAN_deg     = Number(document.getElementById("maxPan").value);
  VELPAN         = Number(document.getElementById("velPan").value);
  SW_MINPAN_deg  = Number(document.getElementById("sw_minPan").value);
  SW_MAXPAN_deg  = Number(document.getElementById("sw_maxPan").value);
  SW_VELPAN      = Number(document.getElementById("sw_velPan").value);
  SW_MINTILT_deg = Number(document.getElementById("sw_minTilt").value);
  SW_MAXTILT_deg = Number(document.getElementById("sw_maxTilt").value);
  SW_VELTILT     = Number(document.getElementById("sw_velTilt").value);
  settingOverlay();
}

var servoType;
var productName;
var productID;
var HWVersion;
var AppVersion;
var URL;
var Email;
var Tel;

function About()
{
  servoType=robInfo.i.desc;
  productName = robInfo.i.product_name;
  productID=robInfo.i.product_id;
  HWVersion=robInfo.i.version_string;
  AppVersion=robInfo.i.version_string;
  URL= "www.roadnarrows.com/";
  Email="support@roadnarrows.com";
  Tel="+1.800.275.9568";

  aboutOverlay();
}


function aboutOverlay()
{
  el = document.getElementById("overlay");
  el.style.visibility = (el.style.visibility == "visible")? "hidden":"visible";
  d3.select("#overlay").selectAll("p").style("background-image", "url('img/PanTiltLogo.png')").style("background-repeat", "no-repeat").style("background-position-y", "20px").html( 
            "<h2>" + servoType + "</h2><br/>" + 
            "Product:     " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;" + productName            + "<br/>" + 
            "Product Id:  " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                              000000" + productID               
            + "<br/>" +
            "HW Version:  " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"                               + HWVersion   
            + "<br/>" +
            "App Version: " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"                               + AppVersion  
            
            + "<br/>" +
            "URL:         " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"+ URL                     
            + "<br/>" +
            "Email:       " + "&nbsp; &nbsp;" + Email       
            + "<br/>" +
            
            "Tel:         " + "&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;                                &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;"+ Tel                     + "<br/> </br/> <br/> " +

            "The RoadNarrows Pan-Tilt robotic mechanism is designed and developed by RoadNarrows, a robotics and intelligent systems company based in Colorado USA. We are dedicated to supporting open software and hardware interfaces to foster a global community of users and developers.");
}

function ClearAlarms()
{
  console.info("Clear Alarms");
  d3.select("box").text("pan-tilt alarms cleared.");
  data[0]["Alarms"] = "OK";
  data[1]["Alarms"] = "OK";
  pan.clearAlarms();
}

d3.select("main").select("table").text("here")

//global table variables 
var columns;
var tbl;
var table;
var thead;
var tbody;
var th;
var rows;
var cells;
var data;

function Table() {      
  columns = ['Joint', 'ServoID', 'State', 'Position', 'Odometer', 'Encoder', 'Velocity', 'Speed', 'Effort', 'Temperature', 'Voltage', 'Alarms'];
table = d3.select("main").append('table');
 data =[
        {"Joint"      : "pan:", 
         "ServoID"    : "na", 
         "State"      : "na", 
         "Position"   : "na", 
         "Odometer"   : "na", 
         "Encoder"    : "na", 
         "Velocity"   : "na", 
         "Speed"      : "na", 
         "Effort"     : "na", 
         "Temperature": "na", 
         "Voltage"    : "na", 
         "Alarms"     : "OK"  },

        {"Joint"      : "tilt:", 
         "ServoID"    : "na", 
         "State"      : "na", 
         "Position"   : "na", 
         "Odometer"   : "na", 
         "Encoder"    : "na", 
         "Velocity"   : "na", 
         "Speed"      : "na", 
         "Effort"     : "na", 
         "Temperature": "na", 
         "Voltage"    : "na", 
         "Alarms"     : "OK"  }
        ];
d3.select("table").call(t);

}

d3.table = function(config) {

  tbl = function(selection) {
    if(columns.length == 0)
    { 
      columns = d3.keys(selection.data()[0][0]);
      console.info(columns);
    }

  table.selectAll('thead').data([0]).enter().append('thead');
  thead = table.select('thead');

  table.selectAll('tbody').data([0]).enter().append('tbody');
  tbody = table.select('tbody');

  th = thead.append("tr").selectAll("th")
     .data(columns)

  th.enter().append("th");
  th.text(function(d) { return d;})
  th.exit().remove();

 

  rows = tbody.selectAll("tr")
      .data(data) 

  rows.enter().append("tr");
  rows.exit().remove();

  cells = rows.selectAll("td")
      .data(function(row) 
           {
             return columns.map(function(key) 
                               {
                                 return {key:key, value:row[key]};
                               });
           })
  cells.enter().append("td");
  cells.text(function(d) { return d.value; })
      .attr('data-col', function(d,i){ return i})
      .attr('data-key', function(d,i){(d.key); return d.key});
  cells.exit().remove();
  return tbl;
  };

  tbl.columns = function(_) {
    if(!arguments.length) { return columns;}
    columns = _;
    return this;
  };

  return tbl;


};

var t= d3.table();

 

function updateData() {
  data[0]["ServoID"]    = robotState.servo_health[0].servo_id;
  data[0]["Temperature"]= robotState.servo_health[0].temp;
  data[0]["Voltage"]    = robotState.servo_health[0].voltage.toFixed(1);
  data[0]["Effort"]     = jointState.effort[0];
  data[0]["Speed"]      = jointState.raw_speed[0];
  data[0]["Odometer"]   = jointState.odometer_pos[0];
  data[0]["Encoder"]    = jointState.encoder_pos[0];
  data[0]["Position"]   = ((180/Math.PI)*(jointState.position[0])).toFixed(2);
  data[0]["Velocity"]   = jointState.velocity[0].toFixed(2);
  //data[0]["Alarms"]     = "Alarmed"


  data[1]["ServoID"]    = robotState.servo_health[1].servo_id;
  data[1]["Temperature"]= robotState.servo_health[1].temp;
  data[1]["Voltage"]    = robotState.servo_health[1].voltage.toFixed(1);
  data[1]["Effort"]     = jointState.effort[1];
  data[1]["Speed"]      = jointState.raw_speed[1];
  data[1]["Odometer"]   = jointState.odometer_pos[1];
  data[1]["Encoder"]    = jointState.encoder_pos[1];
  data[1]["Position"]   = ((180/Math.PI)*(jointState.position[1])).toFixed(2);
  data[1]["Velocity"]   = jointState.velocity[1].toFixed(2);
  //data[1]["Alarms"]     = "Alarmed"

  var i;  
  for(i = 0; i<2; i++)  
  {        
    if(jointState.op_state[i].calib_state == 0)
    {
      data[i]["State"]="uncalibrated";
    }
    else if(jointState.op_state[i].calib_state == 1)
    {
      data[i]["State"]="calibrating";
    }          
    else if(jointState.op_state[0].calib_state == 2 && jointState.op_state[1].calib_state == 2)
    {
      data[i]["State"]="calibrated";
      //set RobotStatus to calibrated
      d3.select("#cancelCalibrate")
          .text("Calibrate")
          .style("background-color","#ccc")
          .style("background-image", "url('img/Calibrate.png')")
          .attr("id","Calibrate")
          .attr("onclick","Calibrate()");
    }
  }
  
  for(i = 0; i<2; i++)
  {
    if(robotState.servo_health[i].alarm == 0)
      data[i]["Alarms"] = "OK";
    else
    {
      data[1]["Alarms"] = "Alarmed";
      fifth.style("background-image", "url('img/red.png')");
    }   
  } 
  
  if(robotState.mode.val == 1)
  {
    first.text("Mode: Manual")
  }
  else
  {
    first.text("Mode: Auto")
  }

  if(robotState.is_calibrated.val == 0)
  {
    second.text("State: Not Calibrated");
    d3.select("#ZeroPoint")
          .style("background-color", "#ccc")
          .attr("onclick",null); 
    d3.select("#Pan")
          .style("background-color", "#ccc")
          .attr("onclick",null); 
    d3.select("#Sweep")
          .style("background-color", "#ccc")
          .attr("onclick",null); 
    d3.select("#SpecifyMove")
          .style("background-color", "#ccc")
          .attr("onclick",null); 
  }
  else
  {
    second.text("State: Calibrated");
   d3.select("#ZeroPoint")
          .attr("onclick","ZeroPoint()"); 
    d3.select("#Pan")
          .attr("onclick","Pan()"); 
    d3.select("#Sweep")
          .attr("onclick","Sweep()"); 
    d3.select("#SpecifyMove")
          .attr("onclick","SpecifyMove()");
  }

  
  if(robotState.motion_possible.val > 0)
  {
    third.style("background-image", "url('img/green.png')")
  }
  else
  {
    third.style("background-image", "url('img/dark.png')")
  }

  if(robotState.in_motion.val > 0)
  {
    fourth.style("background-image", "url('img/green.png')")
  }
  else
  {
    fourth.style("background-image", "url('img/dark.png')")
  }

  if(robotState.errorcode > 0)
  {
    fifth.style("background-image", "url('img/green.png')")
  }
  else
  {
    fifth.style("background-image", "url('img/dark.png')")
  }
  
  if(robotState.e_stopped.val > 0)
  {
    sixth.style("background-image", "url('img/red.png')")
    fifth.style("background-image", "url('img/red.png')")
  }
  else
  {
    sixth.style("background-image", "url('img/dark.png')")
  }


  rows.data(data)

  cells.data(function(row) 
            {
              return columns.map(function(key) 
                                {
                                  return {key:key, value:row[key]};
                                });
            })
      .text(function(d) { return d.value; });
  
};
       



