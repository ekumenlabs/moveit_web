var model, urdf, viewer;
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
  function addGoal_(pose) {
    console.log('goal pose: ', pose);
    position = pose.position;
    var geometry = new THREE.SphereGeometry(0.03,0.03,0.03);
    var material = unknownColor;
    currentGoal = new THREE.Mesh( geometry, material );
    currentGoal.position.set(position.x, position.y, position.z);
    currentGoal.pose = pose;
    viewer.scene.add(currentGoal);
    goals.push(currentGoal);
  }

  // --------------- SCENES -------------------------
  var currentScene;
  function loadScene(sceneId) {
    if(sceneId == 0) {
      // This would be a database object describing the scene
      currentScene = {
        id: 0,
        name: 'A Table',
        object: {
          id: 0,
          name: 'table 0',
          meshUrl: '/static/meshes/table_4legs.dae',
          pose: {
            position: { x: 0.5, y: -1, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 0 }
          }
        }
      };
    } else {
      currentScene = {
        id: 1,
        name: 'Blank',
        object: null
      };
    }

    renderScene();

    // NOTE: The scene is set on the back-end regardless
    // of the result of loading the client scene
    plan.emit('scene_changed', currentScene);
  }
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
      loadScene(Number(picker.selectedOptions[0].value));
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

    // Load new meshes and render them
    // TODO: Materials ???
    currentScene.objects.forEach(function(object) {
      var loader = new ColladaLoader2();
      loader.load(object.meshUrl, function(dae){
        var scene = dae.scene;
        mixin(scene.position, object.pose.position);
        // TODO: Use Quaternion
        mixin(scene.rotation, object.pose.orientation);
        viewer.scene.add(scene);
        _sceneObjects.push(scene);
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
