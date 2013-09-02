/**
 * An Axes object can be used to display the axis of a particular coordinate frame.
 *
 * @constructor
 * @param options - object with following keys:
 *   * shaftRadius (optional) - the radius of the shaft to render
 *   * headRadius (optional) - the radius of the head to render
 *   * headLength (optional) - the length of the head to render
 */
Axes = function(options) {
  var that = this;
  options = options || {};
  var shaftRadius = options.shaftRadius || 0.008;
  var headRadius = options.headRadius || 0.023;
  var headLength = options.headLength || 0.1;
  var longLine = options.longLine || .5;
  var shortLine = options.shortLine || (longLine * .5);
  var scalarFactor = options.scalarFactor || 1;

  THREE.Object3D.call(this);

  // create the cylinders for the objects
  this.headGeom = new THREE.CylinderGeometry(0, headRadius, headLength);

  function addAxis(axis, isLong) {
    isLong = isLong || false;

    var lengthFactor = shortLine;
    var colorHex = axis.y ? 0x00ff00 : 0x0000ff;

    var shaftLength = (shortLine - headLength) * scalarFactor;
    if (isLong) {
      var lengthFactor = longLine;
      var colorHex = 0xff0000;
      var shaftLength = (longLine - headLength) * scalarFactor;
    }
    // set the color of the axis
    var material = new THREE.MeshBasicMaterial({ color : colorHex });

    // setup the rotation information
    var rotAxis = new THREE.Vector3();
    rotAxis.crossVectors(axis, new THREE.Vector3(0, -1, 0));
    var rot = new THREE.Quaternion();
    rot.setFromAxisAngle(rotAxis, 0.5 * Math.PI);

    // create the arrow
    var arrow = new THREE.Mesh(that.headGeom, material);
    arrow.position = axis.clone();
    arrow.position.multiplyScalar(lengthFactor * 0.95);
    arrow.useQuaternion = true;
    arrow.quaternion = rot;
    arrow.updateMatrix();
    that.add(arrow);

    // create the line
    var lineGeom = new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength * scalarFactor);
    var line = new THREE.Mesh(lineGeom, material);
    line.position = axis.clone();
    line.position.multiplyScalar(lengthFactor * 0.43);
    line.useQuaternion = true;
    line.quaternion = rot;
    line.updateMatrix();
    that.add(line);
  }

  // add the three markers to the axes
  addAxis(new THREE.Vector3(1, 0, 0), true);
  addAxis(new THREE.Vector3(0, 1, 0));
  addAxis(new THREE.Vector3(0, 0, 1));
};
Axes.prototype.__proto__ = THREE.Object3D.prototype;


///////////////////////////////////////////////////////////////////////////////

// @constructor
//
// @param pose (optional) - ROSLIB.Pose object to call setPose() with.
// @param scene (optional) - THREE.Scene object (as in ``new ROS3D.Viewer().scene``) to add the goal to.
MoveItGoal = function (pose, scene) {
  // Signal THREE.js this objects use quaternions to express local rotations
  this.useQuaternion = true;

  // contruct the small dot to show the position
  var dot_mesh = new THREE.Mesh(
    MoveItGoal.dotGeometry,
    MoveItGoal.statusMaterials.unknown
  );
  this.add(dot_mesh);

  // construct the axes, and keep a reference to show rotations
  var axes = new Axes();
  this.add(axes);

  // add this goal to the general repository
  MoveItGoal.all.push(this);

  // set position and orientation, if given
  pose && this.setPose(pose);

  // add to the scene, if given
  scene && scene.add(this);
}

// One repository for all goals created.
//
// TODO: allow to remove this goals from the scene and deallocate them from
// memory as a one "clearAll()" call.
MoveItGoal.all = [];

MoveItGoal.dotGeometry = new THREE.SphereGeometry(0.03,0.03,0.03);

MoveItGoal.statusMaterials = {
  unknown: new THREE.MeshBasicMaterial({ color: 0xffffff }),
  reachable: new THREE.MeshBasicMaterial({ color: 0x00ff00 }),
  unreachable: new THREE.MeshBasicMaterial({ color: 0xff0000 })
};

// Returns a reference to the last goal created, null if there's none.
MoveItGoal.latest = function () {
  var len = MoveItGoal.all.length;
  if (len > 0)
    return MoveItGoal.all[len - 1];
  return null;
}

MoveItGoal.prototype = new THREE.Object3D();

// Set both position and orientation (using a quaternion).
MoveItGoal.prototype.setPose = function (pose) {
  this.setPosition(pose.position);
  this.setOrientation(pose.orientation);

  // Keep a reference to the original pose to serialize
  this.pose = pose;
}

MoveItGoal.prototype.setPosition = function (position) {
  // NOTE that a ROSLIB.Vector3 is not a THREE.Vector3... it doesn't even share
  // a prototype, _as it should_. Check out the code and see.
  this.position.x = position.x;
  this.position.y = position.y;
  this.position.z = position.z;
}

MoveItGoal.prototype.setOrientation = function (orientation) {
  // NOTE that a ROSLIB.Quaternion is not a THREE.Quaternion... it doesn't even
  // share a prototype, _as it should_. Check out the code and see.
  this.quaternion = new THREE.Quaternion(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
  );
}
