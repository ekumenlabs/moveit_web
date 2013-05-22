
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

  // Keep track of scene dragging
  this.lastX = 0;
  this.lastY = 0;
  this.mouseDown = false;
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
  group.add(table);

  group.position.z = .5;

  // Add the poses

  this.setObject3D(group);
}

TABLE.POSES = [{
  id: 0,
  reachable: 0,
  position: {x: -0.5, y: -0.5, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0}
}, {
  id: 1,
  reachable: 0,
  position: {x: 0, y: -0.5, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 2,
  reachable: 0,
  position: {x: 0.5, y: -0.5, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 3,
  reachable: 0,
  position: {x: -0.5, y: 0, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 4,
  reachable: 0,
  position: {x: 0, y: 0, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 5,
  reachable: 0,
  position: {x: 0.5, y: 0, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 6,
  reachable: 0,
  position: {x: -0.5, y: 0.5, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 7,
  reachable: 0,
  position: {x: 0, y: 0.5, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}, {
  id: 8,
  reachable: 0,
  position: {x: 0.5, y: 0.5, z: 1},
  orientation: {x: 0, y: 0, z: 0, w: 0},
}];
