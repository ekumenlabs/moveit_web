$(function(){
  console.log("starting");

  // Create viewer
  var viewer = new ROS3D.Viewer({
    divID : 'canvas',
    width : window.innerWidth,
    height : window.innerHeight,
    antialias : true
  });
  // Add grid
  viewer.addObject(new ROS3D.Grid());

  $.get('/static/pr2_description/pr2.urdf',function(urdf_string){
    // Create URDF object
    var urdfModel = new ROSLIB.UrdfModel({
      string: urdf_string
    });
    var ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });
    var tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0
    });
    var theURDF = new ROS3D.Urdf({
      urdfModel: urdfModel,
      path: 'http://resources.robotwebtools.org/',
      tfClient: tfClient
      /* path: '/static/',
      tfClient: {
        subscribe: function() {}
      } */
    });
    viewer.addObject(theURDF);
    console.log("done");
  });
});
