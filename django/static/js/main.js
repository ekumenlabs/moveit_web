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
  $('#random').on('click',function(ev){
    plan.emit('goal_random');
  });
  $('#run').on('click',function(ev){
    plan.emit('plan_to_poses', [MoveItGoal.latest().pose]);
    $('#run').attr('disabled','disabled');
    $('#run').html('working ...');
  });
  $('#clear').on('click',function(ev){
    MoveItGoal.all.forEach(function(goal){
      viewer.scene.remove(goal);
    });
  });
  $('#scene-load').on('click',function(ev){
    selectScene();
  });
  $('#robot-load').on('click',function(ev){
    alert("Not implemented");
  });

  // Socket.io events
  var plan = io.connect('/plan');
  plan.on('status',function(statusMessage){
    if('text' in statusMessage) {
       $('#status-message').html(statusMessage['text']);
    }
    if('reachable' in statusMessage) {
      MoveItGoal.latest().material = statusMessage['reachable']?reachableColor:unreachableColor;
    }
    if('ready' in statusMessage && statusMessage['ready']) {
      $('#run').removeAttr('disabled');
      $('#run').html('Run');
    }
  });
  plan.on('target_pose', function(pose){
    console.log('Will plan to position: ', pose);
    addGoal( pose)
  });
  plan.on('link_poses',function(poses){
    updatePoses( poses);
  });
  plan.on('connect',function() {
    plan.emit('connected');
  });
  plan.on('current scene',function(scene) {
    currentScene = scene;
    renderScene();
  });
  plan.on('scene changed', loadScene);

  // -------------- GOALS ---------------------------
  $('#goal-make').on('click',function(ev){
    ev.preventDefault();
    var i = eval('({'+$('#goal-data').val()+'})');
    var pose = {
      position: {
       x: i.xyz[0], y: i.xyz[1], z: i.xyz[2]
      }, orientation: {
       x: i.q[0], y: i.q[1], z: i.q[2], w: i.q[3]
      }
    }
    addGoal(pose);
  });

  var unknownColor = new THREE.MeshBasicMaterial( { color: 0xffffff } );
  var reachableColor = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
  var unreachableColor = new THREE.MeshBasicMaterial( { color: 0xff0000 } );
  var goals = [];
  var currentGoal;
  function addGoal(pose) {
    var goal = new MoveItGoal(pose, viewer.scene);
  }

  // --------------- SCENES -------------------------
  //
  // 1. User selects a scene from the menu
  // 2. Browser emits 'change scene' with the name of the new scene
  // 3. Server queries DB for scene with that name
  // 4. Server sets the scene in MoveIt! backend
  // 5. Server emits 'scene changed' with serialized data of the scene.
  //    The scene data includes:
  //      * the name (which is known to the client)
  //      * the thumbnail URL (which is known to the client)
  //      * the mesh URL
  //      * the pose, as a JSON object
  var currentScene; // this global might not be necessary anymore.

  // React to a scene change.
  function loadScene(scene) {
    currentScene = scene;
    renderScene();
  }
  // Produce a scene change.
  function selectScene() {
    $('#canvas').hide();
    $('#scene-choose').show();
    var picker = $('#scene-select').
      imagepicker({
        hide_select: true,
        selected: function(ev, a) {
          $('#scene-select-name').html(
            picker.selectedOptions[0].text
          );
        }
      })[0];
    $('#scene-select-done').one('click',function(ev){
      $('#scene-choose').hide();
      $('#canvas').show();
      var sceneName = picker.selectedOptions[0].value;
      plan.emit('change scene', sceneName);
    });
  }

  // HACK: Low-level-ish: keep track of which objects we added to the scene
  // as 'environment' (as opposed to the robot) so that we can remove them
  // when we change from one scene to the next
  var _sceneObjects = [];
  function renderScene() {
    $('#scene-name').html(currentScene.name);

    // Remove previosly-added scene objects
    _sceneObjects.forEach(function(object) {
      viewer.scene.remove(object);
    });
    _sceneObjects = [];

    // Load new meshes and render them
    // TODO: Materials ???
    if (currentScene.meshUrl) {
      var loader = new ColladaLoader2();
      loader.load(currentScene.meshUrl, function(dae){
        var pose = currentScene.pose;
        var scene = dae.scene;

        // TODO: Use Quaternion
        mixin(scene.rotation, pose.orientation);
        mixin(scene.position, pose.position);

        viewer.scene.add(scene);
        _sceneObjects.push(scene);
      });
    }
  }

  // -------------- UTILS --------------
  function mixin(into, what) {
    for(key in what) {
      into[key] = what[key];
    }
  }
});
