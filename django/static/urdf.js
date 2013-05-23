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
    var theURDF = new ROS3D.Urdf({
      urdfModel: urdfModel,
      // path: '/static/',
      path: 'http://resources.robotwebtools.org/',
      tfClient: {
        subscribe: function() {}
      }
    });
    viewer.addObject(theURDF);
    console.log("done");
  });
});
