var model, urdf, table, viewer;
$(function(){
  console.log("starting");

  // Viewer with grid
  viewer = new ROS3D.Viewer({
    divID : 'canvas',
    width : $('#canvas').width(),
    height : 360,
    //height : window.innerHeight - 140,
    antialias : true
  });
  viewer.addObject(new ROS3D.Grid());

  // --------------- ROBOT ----------------------
  $.get('/static/pr2_description/pr2.urdf',function(urdf_string){
    // Create URDF object
    var urdfModel = new ROSLIB.UrdfModel({
      string: urdf_string
    });
    model = urdfModel;
    var theURDF = new ROS3D.Urdf({
      urdfModel: urdfModel,
      path: 'http://resources.robotwebtools.org/',
      tfClient: {
        subscribe: function() {}
      } 
    });
    urdf = theURDF;
    viewer.addObject(theURDF);
    console.log("done");
  });
  function updatePoses(poses) {
    var linkNameToChildIndexMap = {};
    urdf.children.forEach(function(sceneNode){
      linkNameToChildIndexMap[sceneNode.frameID] = sceneNode;
    });
    poses.global_link.forEach(function(poseAsList){
      var linkName = poseAsList[0];
      if('/'+linkName in linkNameToChildIndexMap) {
        var sceneNode = linkNameToChildIndexMap['/'+linkName];
        sceneNode.updatePose({
          position: { x: poseAsList[1], y:  poseAsList[2], z:  poseAsList[3] },
          orientation: { x: poseAsList[4], y: poseAsList[5], z: poseAsList[6], w: poseAsList[7] }
        });
      } else {
      }
    });
  }

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
  $('#scene').on('click',function(ev){
    loadScene();
  });
  $('#deleteme_test').on('click',function(ev){
    // plan.emit('deleteme_test');
    plan.emit('scene_changed', currentScene);
  });

  // Socket.io events
  var plan = io.connect('/plan');
  var initialPoseRequested = false;
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
      if(!initialPoseRequested) {
        plan.emit('get_link_poses');
        initialPoseRequested = true;
      }
    }
  });
  plan.on('target_pose',function(position){
    console.log('Will plan to position: ', position);
    addGoal( position)
  });
  plan.on('link_poses',function(poses){
    updatePoses( poses);
  });
  plan.on('connect',function() {
    plan.emit('connected');
  });

  // -------------- GOALS ---------------------------

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

  // --------------- SCENES -------------------------
  var currentScene;
  function loadScene() {
    // This would be a database object describing the scene
    var sceneHardcoded = {
      id: 0,
      name: 'A Table',
      objects: [{
        id: 0,
        name: 'table 0',
        meshUrl: '/static/meshes/table_4legs.dae',
        pose: {
          position: { x: 0.5, y: -1, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 0 }
        }
      },{
        id: 1,
        name: 'table 1',
        meshUrl: '/static/meshes/table_4legs.dae',
        pose: {
          position: { x: 0.5, y: -2.5, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 0 }
        }
      },{
        id: 2,
        name: 'table 2',
        meshUrl: '/static/meshes/table_4legs.dae',
        pose: {
          position: { x: 0.5, y: -2.5, z: 0 },
          orientation: { x: 0, y: 0, z: 3.14159/2.0, w: 0 }
        }
      }]
    };
    currentScene = sceneHardcoded;
    $('#scene-name').html(currentScene.name);

    // Load meshes and render them
    // TODO: Materials ???
    sceneHardcoded.objects.forEach(function(object) {
      var loader = new ColladaLoader2();
      loader.load(object.meshUrl, function(dae){
        var scene = dae.scene;
        table = scene;
        mixin(scene.position, object.pose.position);
        // TODO: Use Quaternion
        mixin(scene.rotation, object.pose.orientation);
        viewer.scene.add(scene);
      });
    });
  }


  // -------------- UTILS --------------
  function mixin(into, what) {
    for(key in what) {
      into[key] = what[key];
    }
  }
});
