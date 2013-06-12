var model, urdf;
$(function(){
  console.log("starting");

  // Create viewer
  var viewer = new ROS3D.Viewer({
    divID : 'canvas',
    width : $('#canvas').width(),
    height : 360,
    //height : window.innerHeight - 140,
    antialias : true
  });
  // Add grid
  viewer.addObject(new ROS3D.Grid());

  $.get('/static/pr2_description/pr2.urdf',function(urdf_string){
    // Create URDF object
    var urdfModel = new ROSLIB.UrdfModel({
      string: urdf_string
    });
    model = urdfModel;
    /*
    var ros = new ROSLIB.Ros({
      url: 'ws://'+window.document.domain+':9090'
    });
    var tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0
    }); */
    var theURDF = new ROS3D.Urdf({
      urdfModel: urdfModel,
      path: 'http://resources.robotwebtools.org/',
      /* tfClient: tfClient
      path: '/static/', */
      tfClient: {
        subscribe: function() {}
      } 
    });
    urdf = theURDF;
    viewer.addObject(theURDF);
    console.log("done");
  });

  // Hook run button to start demo
  $('#run').on('click',function(ev){
    plan.emit('plan_random');
    $('#run').attr('disabled','disabled');
    $('#run').html('working ...');
  });
  $('#clear').on('click',function(ev){
    goals.forEach(function(goal){
      viewer.scene.remove(goal);
    });
  });

  // Socket.io events
  var plan = io.connect('/plan');
  plan.on('status',function(statusMessage){
    if('text' in statusMessage) {
       $('#previous-status-message').html($('#status-message').html());
       $('#status-message').html(statusMessage['text']);
    }
    if('reachable' in statusMessage) {
      lastGoal.material = statusMessage['reachable']?reachableColor:unreachableColor;
    }
    if('ready' in statusMessage && statusMessage['ready']) {
      $('#run').removeAttr('disabled');
      $('#run').html('Random goal');
      plan.emit('get_link_poses');
    }
  });
  plan.on('target_pose',function(position){
    console.log('Will plan to position: ', position);
    addGoal( position)
  });
  plan.on('link_poses',function(poses){
    console.log('Got poses: ', poses);
    updatePoses( poses);
  });
  plan.on('connect',function() {
    plan.emit('connected');
  });

  var unknownColor = new THREE.MeshBasicMaterial( { color: 0xffffff } );
  var reachableColor = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
  var unreachableColor = new THREE.MeshBasicMaterial( { color: 0xff0000 } );

  var goals = [];
  var lastGoal;
  function addGoal(position) {
    var geometry = new THREE.SphereGeometry(0.03,0.03,0.03);
    var material = unknownColor;
    lastGoal = new THREE.Mesh( geometry, material );
    lastGoal.position.set(position.x, position.y, position.z);
    viewer.scene.add(lastGoal);
    goals.push(lastGoal);
  }
  function updatePoses(poses) {
    var linkNameToChildIndexMap = {};
    urdf.children.forEach(function(sceneNode){
      linkNameToChildIndexMap[sceneNode.frameID] = sceneNode;
    });
    poses.global_link.forEach(function(poseAsList){
      var linkName = poseAsList[0];
      if('/'+linkName in linkNameToChildIndexMap) {
        console.log('updating link: ' + linkName);
        var sceneNode = linkNameToChildIndexMap['/'+linkName];
        sceneNode.updatePose({
          position: { x: poseAsList[1], y:  poseAsList[2], z:  poseAsList[3] },
          orientation: { x: poseAsList[4], y: poseAsList[5], z: poseAsList[6], w: poseAsList[7] }
        });
      } else {
        console.log('missed link: ' + linkName);
      }
    });
  }
});
