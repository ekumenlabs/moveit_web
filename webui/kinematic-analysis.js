
KinematicAnalysisApp = function () {

  Sim.App.call(this);

}

KinematicAnalysisApp.prototype = new Sim.App();

KinematicAnalysisApp.prototype.init = function (params) {

  // Borrow Sim.App initializer code to set up scene, renderer, default camera
  Sim.App.prototype.init.call(this, params);

  // Create a directional light to show off the model
  var light = new THREE.DirectionalLight( 0xffffff, 1);
  light.position.set(0, -1, 1).normalize();
  this.scene.add(light);

  this.camera.position.set(0, 0, 15);

  // Add a grid as a floor for the scene
  var floor = new Floor();
  floor.init();
  this.addObject(floor);

  // Create the table in the center
  var table = new Table();
  table.init();
  this.addObject(table);

  TABLE = table // DEBUG

  // Keep track of scene dragging
  this.lastX = 0;
  this.lastY = 0;
  this.mouseDown = false;

  // Initialize a proxy to the MoveIt! backend
  this.moveit = MoveitBackend({
    url: '',

    // UGLY UGLY UGLY
    update: function (poses) { table.children[0].updatePoses(poses) }
  });
}


KinematicAnalysisApp.prototype.handleMouseDown = function(x, y) {
  this.lastX = x;
  this.lastY = y;
  this.mouseDown = true;
}

KinematicAnalysisApp.prototype.handleMouseUp = function(x, y) {
  this.lastX = x;
  this.lastY = y;
  this.mouseDown = false;
}

KinematicAnalysisApp.prototype.handleMouseMove = function(x, y) {
  if (this.mouseDown) {
    // Move the whole scene around the Y axis.
	var dx = x - this.lastX;
	if (Math.abs(dx) > KinematicAnalysisApp.MOUSE_MOVE_TOLERANCE) {
	  this.root.rotation.y += (dx * 0.01);

	  // Clamp to some outer boundary values
	  if (this.root.rotation.y < KinematicAnalysisApp.MIN_ROTATION_Y)
	    this.root.rotation.y = KinematicAnalysisApp.MIN_ROTATION_Y;

	  if (this.root.rotation.y > KinematicAnalysisApp.MAX_ROTATION_Y)
	    this.root.rotation.y = KinematicAnalysisApp.MAX_ROTATION_Y;
	}
	this.lastX = x;

    // Move the whole scene around the X axis.
	var dy = y - this.lastY;
	if (Math.abs(dy) > KinematicAnalysisApp.MOUSE_MOVE_TOLERANCE) {
	  this.root.rotation.x += (dy * 0.01);

	  // Clamp to some outer boundary values
	  if (this.root.rotation.x < KinematicAnalysisApp.MIN_ROTATION_X)
	    this.root.rotation.x = KinematicAnalysisApp.MIN_ROTATION_X;

	  if (this.root.rotation.x > KinematicAnalysisApp.MAX_ROTATION_X)
	    this.root.rotation.x = KinematicAnalysisApp.MAX_ROTATION_X;
	}
	this.lastY = y;
  }
}

KinematicAnalysisApp.prototype.handleMouseScroll = function(delta) {
  var dx = delta;

  this.camera.position.z -= dx;

  // Clamp to some boundary values
  if (this.camera.position.z < KinematicAnalysisApp.MIN_CAMERA_Z)
	this.camera.position.z = KinematicAnalysisApp.MIN_CAMERA_Z;
  if (this.camera.position.z > KinematicAnalysisApp.MAX_CAMERA_Z)
	this.camera.position.z = KinematicAnalysisApp.MAX_CAMERA_Z;
}

KinematicAnalysisApp.MOUSE_MOVE_TOLERANCE = 2;
KinematicAnalysisApp.MAX_ROTATION_X = Math.PI / 2;
KinematicAnalysisApp.MAX_ROTATION_Y = Math.PI / 2;
KinematicAnalysisApp.MIN_ROTATION_X = -Math.PI / 2;
KinematicAnalysisApp.MIN_ROTATION_Y = -Math.PI / 2;
KinematicAnalysisApp.MIN_CAMERA_Z = 4;
KinematicAnalysisApp.MAX_CAMERA_Z = 12;



//
// A fixed grid floor
//

Floor = function () {
  return Sim.Object.call(this);
}
Floor.prototype = new Sim.Object();

Floor.prototype.init = function (params) {
  this.setObject3D(new THREE.Object3D());

  var geometry = new THREE.PlaneGeometry(10, 10, 10, 10)
    , material = new THREE.MeshBasicMaterial({
        color: 0x010101,
        opacity: .2,
        transparent: true,
        wireframe: true,
        wireframeLinewidth: 2
    })
    , floor = new THREE.Mesh(geometry, material);

  this.object3D.add(floor);
}


//
// A fixed table at the center on the floor
//

Table = function () {
  return Sim.Object.call(this);
}
Table.prototype = new Sim.Object();

Table.prototype.init = function (params) {
  var group = new THREE.Object3D
    , geometry = new THREE.CubeGeometry(2, 2, 1, 32, 32, 32)
    , material = new THREE.MeshPhongMaterial({ color: 0x101510 , wireframe: false})
    , table = new THREE.Mesh(geometry, material);

  this.setObject3D(group);

  // Move the table to level with the floor
  table.position.z = .5;
  group.add(table);

  // Add the poses, leveled with the table
  var poses = new PoseGroup();
  poses.init(Table.POSES);

  poses.object3D.position.z = 1;

  this.addChild(poses);
}

Table.POSES = [{
  id: 0,
  reachable: 0,
  position: {x: -0.5, y: -0.5, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0}
}, {
  id: 1,
  reachable: 0,
  position: {x: 0, y: -0.5, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 2,
  reachable: 0,
  position: {x: 0.5, y: -0.5, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 3,
  reachable: 0,
  position: {x: -0.5, y: 0, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 4,
  reachable: 0,
  position: {x: 0, y: 0, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 5,
  reachable: 0,
  position: {x: 0.5, y: 0, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 6,
  reachable: 0,
  position: {x: -0.5, y: 0.5, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 7,
  reachable: 0,
  position: {x: 0, y: 0.5, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 8,
  reachable: 0,
  position: {x: 0.5, y: 0.5, z: 0},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}];


//
// A pose set, all grouped together
//

PoseGroup = function () {
  return Sim.Object.call(this);
}
PoseGroup.prototype = new Sim.Object();

PoseGroup.prototype.init = function (poses) {
  // Group all poses in a single object
  this.setObject3D(new THREE.Object3D());

  // Keep a reference, by ID, to the pose objects in the group
  this.poses = {};

  // Let each point add itself to the group
  for (var i = 0; i < poses.length; i++) {
    var point = new PosePoint()
      , pose = poses[i];

    point.init({root: this.object3D, data: pose});

    this.poses[pose.id] = point;
  }
}

PoseGroup.prototype.updatePoses = function (newPoses) {
  newPoses = newPoses || [];

  for (var i = 0; i < newPoses.length; i++) {
    var newPose = newPoses[i]
      , point = this.poses[newPose.id];

    if (point.reachable !== newPose.reachable) {
      console.log('Updating pose ID=' + newPose.id);

      // Lose the old point
      point.destroy();

      // Create a new point with the new data
      point = new PosePoint();
      point.init({root: this.object3D, data: newPose});

      // Update the point reference
      this.poses[newPose.id] = point;
    }
  }
}

// TEST the pose coloring.
var testUpdatePoses = function () {
  var pose = Table.POSES[4]
  pose.reachable = 1;

  TABLE.children[0].updatePoses([pose]);
}


//
// A single pose object
//

PosePoint = function () {
  return Sim.Object.call(this);
}
PosePoint.prototype = new Sim.Object();

PosePoint.MATERIAL = new THREE.MeshBasicMaterial({color: 0xffffff});
PosePoint.RADIUS = 0.05;
PosePoint.COLOR_MAP = {
    0: new THREE.MeshBasicMaterial({color: 0xffffff})
  , 1: new THREE.MeshBasicMaterial({color: 0x00ff00})
  , 2: new THREE.MeshBasicMaterial({color: 0xff0000})
};

PosePoint.prototype.init = function (params) {
  this.data = params.data;
  this.root = params.root;

  // Data shortcuts
  this.reachable = this.data.reachable;

  // Create the point
  var geometry = new THREE.SphereGeometry(PosePoint.RADIUS, 8, 8)
    , point = new THREE.Mesh(geometry, PosePoint.COLOR_MAP[this.reachable]);

  point.position = params.data.position;

  this.mesh = point;

  this.root.add(point);
}


PosePoint.prototype.destroy = function () {
  this.root.remove(this.mesh);
}


//
// MoveIt! backend proxy
//

MoveitBackend = function (params) {
  // Delegate networking calls to the network manager object
  this.network = new NetworkManager(params);

  // Where to callback to, when the new poses are ready
  this.updateCallback = params.update;
}
MoveitBackend.prototype = {}

MoveitBackend.prototype.go = function () {
  var that = this;

  // The backend knows what to do
  this.network.go(function () {
    // Get the new poses when the "go" call is done
    this.network.getAllPoses(function (poses) {
      // Callback to application code to update the poses
      that.updateCallback(poses);
    });
  });
}


//
// Network-solutions object
//

NetworkManager = function (params) {
  this.endpoint = params.url;
}
NetworkManager.prototype = {}

NetworkManager.prototype.go = function (fn) {
  // Stub implementation

  // Implement AJAX request to the backend to trigger recomputation
  //
  // on complete --> fn()
  fn();
}

NetworkManager.prototype.getAllPoses = function (fn) {
  // Stub implementation

  // Implement the AJAX request to bring the new poses
  //
  // on complete --> fn(newPoses)

  var newPoses = TABLE.POSES // THIS SHOULD BE THE server POSES
    , poses = [];


  // Re-format the returned poses
  for (var i = 0; i < newPoses.length; i++) {
    poses[i] = this.formatPose(newPoses[i]);
  }

  fn(poses);
}

NetworkManager.prototype.formatPose = function (remotePose) {
  var pose = {};
  pose.id = remotePose.id;
  pose.reachable = remotePose.reachable;
  pose.position = remotePose.pose.position;
  pose.orientation = remotePose.pose.orientation;
  return pose;
}
